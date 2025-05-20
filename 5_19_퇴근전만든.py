import sys
import socket
import struct
import pickle
import time
import threading
import queue
import numpy as np
import sip

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from UI.trackbot_gui import Ui_MainWindow  # .ui를 pyuic5로 변환한 파일
from PyQt5.QtGui import QImage, QPixmap, QPainter
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QGraphicsView, QVBoxLayout, QLabel
import image_processing as ip
from PyQt5 import QtGui, QtCore  # 이미 있으면 생략

## Available platform plugins are: xcb, eglfs, linuxfb, 
# minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx.
import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"

from PyQt5.QtWidgets import QVBoxLayout
from box_drawing_view import BoxDrawingView  # 같은 파일에 있으면 생략 가능

class PickleSender:
    def __init__(self, host, port, get_state_callback):
        self.host = host
        self.port = port
        self.get_state_callback = get_state_callback
        self.sock = None
        self.running = False

    def run(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.running = True
                print(f"[TCP-CTRL] 서버 연결됨: {self.host}:{self.port}")

                while self.running:
                    try:
                        state = self.get_state_callback()
                        self.send_pickle_message("control_state", state)
                        time.sleep(0.01)  # 100Hz

                    except Exception as e:
                        print(f"[송신 중 오류] {e}")
                        break

            except Exception as e:
                print(f"[연결 실패] {e}")

            finally:
                self.running = False
                if self.sock:
                    try:
                        self.sock.shutdown(socket.SHUT_RDWR)
                    except:
                        pass
                    self.sock.close()
                    self.sock = None
                print("[TCP-CTRL] 연결 종료됨. 3초 후 재연결 시도...")
                time.sleep(3)

    def send_pickle_message(self, type_str, body_dict):
        try:
            body_dict["type"] = type_str
            pickled_data = pickle.dumps(body_dict)
            header = struct.pack("!I", len(pickled_data))
            self.sock.sendall(header + pickled_data)
        except Exception as e:
            print(f"[피클 전송 오류] {e}")

    def close(self):
        self.running = False
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self.sock.close()
            self.sock = None


class TrackBotApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 이미지 배열 관리를 위한 변수들
        self.image_arrays = [None, None, None]  # 이전, 현재, 다음 이미지 배열
        self.current_image_index = 1  # 현재 보여지는 이미지의 인덱스
        self.image_loaded = [False, False, False]  # 각 이미지의 로드 상태
        self.merged_image = None  # 병합된 이미지
        self.merged_done = False

        # 새로운 커스텀 View로 대체
        self.scene = QGraphicsScene()
        self.image_item = QGraphicsPixmapItem()
        self.scene.addItem(self.image_item)

        self.custom_view = BoxDrawingView()
        self.custom_view.setScene(self.scene)
        self.custom_view.setRenderHint(QPainter.Antialiasing)
        self.custom_view.setDragMode(QGraphicsView.ScrollHandDrag)

        # 기존 UI 위젯 내부에 덮어쓰기
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.custom_view)
        self.ui.LineCamera.setLayout(layout)

        self.saved_boxes = {} 
        self.image_queue = queue.Queue()

        # 저장된 박스 데이터 로드
        self.load_saved_boxes()

        # 주기적으로 큐에서 이미지 확인
        self.image_check_timer = QTimer()
        self.image_check_timer.timeout.connect(self.check_image_queue)
        self.image_check_timer.start(50)  # 50ms마다 체크

        # screen width pix gain
        _height = 4096
        self.gain = _height/self.ui.LineCamera.height()
        self.viewer_width = int(self.gain * self.ui.LineCamera.width())

        #################################################################################
        # 설정파일 불러오기
        inspection_config = ip.load_inspection_config("Config/inspection_config.txt")
        print(inspection_config)

        # 픽셀거리정보 생성
        tot_length = 0
        for idx, data in enumerate(inspection_config):
            # data : image_name,image_path,distance_m,length_pix,total_length
            _length = ip.calculate_required_pixels(data[2],1000)
            inspection_config[idx][3] = _length
            inspection_config[idx][4] = tot_length
            tot_length += _length

        print(inspection_config)

        # save default image 
        for idx, data in enumerate(inspection_config):
            _length = data[3]
            _base_path = "inspection_result/image/"
            _image_path = _base_path + str(idx) + ".jpg"
            ip.generate_white_image_cv(width=_length, height=4096, save_path=_image_path)
            inspection_config[idx][1] = _image_path

        print(inspection_config)

        self.inspection_config = inspection_config

        # 시작 offset, 전체길이, 중간 이미지 인덱스, 사용가능여부, Numpy array
        self.image_data = [0,0,0,False,None]  # 1,2,3 3개를 가지고 있기
        self.update_image_data(1)  # 중간 이미지 인덱스로 초기화

        self.setup_ui_logic()

        # ✅ 녹색 바 오버레이
        self.green_bar_overlay = QLabel(self.ui.centralwidget)
        self.green_bar_overlay.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.green_bar_overlay.setStyleSheet("background: transparent")
        self.green_bar_overlay.raise_()

        # ✅ 빨간 라인 오버레이
        self.red_line_overlay = QLabel(self.ui.centralwidget)
        self.red_line_overlay.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.red_line_overlay.setStyleSheet("background: transparent")
        self.red_line_overlay.raise_()

        # 초기 위치 설정
        self.update_overlay_positions()

        # 초기 이미지 로드
        self.load_initial_images()

        # 🔌 PickleSender 시작
        self.pickle_sender = PickleSender(
            host="ap.loclx.io",
            port=12000,
            get_state_callback=lambda: self.control_state
        )
        threading.Thread(target=self.pickle_sender.run, daemon=True).start()

        #################################################################################

        # 박스가 만들어지거나 수정될 때마다 자동 저장
        self.custom_view.boxUpdated.connect(self.save_boxes_for_image)

    def update_image_data(self, mid_idx):
        """이미지 데이터 업데이트"""
        self.image_data[0] = self.inspection_config[mid_idx-1][4]  # 첫번째 사진의 total_length
        self.image_data[1] = (
            self.inspection_config[mid_idx-1][3] + 
            self.inspection_config[mid_idx][3] + 
            self.inspection_config[mid_idx+1][3]
        )
        self.image_data[2] = mid_idx

    def load_initial_images(self):
        """초기 3개의 이미지를 로드"""
        mid_idx = 1  # 중간 이미지 인덱스
        self.load_requested_image(mid_idx - 1, 0)  # 이전 이미지
        self.load_requested_image(mid_idx, 1)      # 현재 이미지
        self.load_requested_image(mid_idx + 1, 2)  # 다음 이미지

    def load_requested_image(self, idx, array_index):
        """특정 인덱스의 이미지를 로드하여 지정된 배열 위치에 저장"""
        if 0 <= idx < len(self.inspection_config):
            image_path = self.inspection_config[idx][1]
            loader = ip.ImageLoadRequestThread(idx, image_path, 
                lambda i, img: self.handle_loaded_image(i, img, array_index))
            loader.start()
        else:
            # 이미지가 없는 경우 빈 이미지 생성
            self.handle_loaded_image(idx, np.zeros((4096, 100, 3), dtype=np.uint8), array_index)

    def handle_loaded_image(self, idx, image_np, array_index):
        """이미지 로드 완료 처리"""
        self.image_arrays[array_index] = image_np
        self.image_loaded[array_index] = True
        print(f"[✅ 로드 완료] {idx}번째 → 배열 {array_index}에 저장됨")
        
        # 모든 이미지가 로드되었는지 확인
        if all(self.image_loaded):
            self.merge_images()

    def merge_images(self):
        """3개의 이미지를 병합"""
        try:
            self.merged_image = np.concatenate(
                [self.image_arrays[0], self.image_arrays[1], self.image_arrays[2]],
                axis=1
            )
            self.merged_done = True
            print("[✅] 이미지 병합 완료")

            # NumPy → QImage → QPixmap
            height, width, ch = self.merged_image.shape
            bytes_per_line = ch * width
            qimg = QImage(
                self.merged_image.tobytes(), width, height, bytes_per_line, QImage.Format_RGB888
            ).rgbSwapped()
            self.cached_pixmap = QPixmap.fromImage(qimg)
            self.image_data[4] = self.cached_pixmap
            self.image_data[3] = True

        except Exception as e:
            print(f"[❌] 병합 실패: {e}")

    def shift_images(self, direction):
        """이미지 배열을 한 방향으로 이동"""
        if direction == 'right':
            # 오른쪽으로 이동: [0,1,2] -> [1,2,new]
            self.image_arrays = [self.image_arrays[1], self.image_arrays[2], None]
            self.image_loaded = [self.image_loaded[1], self.image_loaded[2], False]
            self.current_image_index += 1
            
            # 새로운 이미지 로드 (마지막 이미지인 경우 빈 이미지)
            if self.current_image_index + 1 < len(self.inspection_config):
                self.load_requested_image(self.current_image_index + 1, 2)
            else:
                self.handle_loaded_image(-1, np.zeros((4096, 100, 3), dtype=np.uint8), 2)
        else:
            # 왼쪽으로 이동: [0,1,2] -> [new,0,1]
            self.image_arrays = [None, self.image_arrays[0], self.image_arrays[1]]
            self.image_loaded = [False, self.image_loaded[0], self.image_loaded[1]]
            self.current_image_index -= 1
            
            # 새로운 이미지 로드 (첫 번째 이미지인 경우 빈 이미지)
            if self.current_image_index - 1 >= 0:
                self.load_requested_image(self.current_image_index - 1, 0)
            else:
                self.handle_loaded_image(-1, np.zeros((4096, 100, 3), dtype=np.uint8), 0)

    def update_overlay_positions(self):
        # 녹색 바는 horizontalScrollBar 위
        green_geom = self.ui.horizontalScrollBar.geometry()
        self.green_bar_overlay.setGeometry(green_geom)

        # 빨간 선은 horizontalScrollBar_2 위
        red_geom = self.ui.horizontalScrollBar_2.geometry()
        self.red_line_overlay.setGeometry(red_geom)

    def check_image_queue(self):
        try:
            image_np = self.image_queue.get_nowait()
            self.handle_loaded_image(image_np)
        except queue.Empty:
            pass

    def display_pixmap_slice(self, pixmap, start_x, width):
        height = pixmap.height()
        max_width = pixmap.width() - start_x
        if max_width < width:
            width = max_width
        sub_pixmap = pixmap.copy(start_x, 0, width, height)

        self.image_item.setPixmap(sub_pixmap)
        self.scene.setSceneRect(0, 0, sub_pixmap.width(), height)
        self.custom_view.fitInView(self.image_item, Qt.IgnoreAspectRatio)

    def load_boxes_for_current_view(self, start_idx):
        """현재 보이는 이미지 영역에 해당하는 박스들을 불러와서 표시"""
        # 현재 보이는 이미지 인덱스들 계산
        mid_idx = self.image_data[2]
        visible_indices = [mid_idx - 1, mid_idx, mid_idx + 1]
        
        # 각 이미지에 대해 저장된 박스들을 불러와서 표시
        for img_idx in visible_indices:
            if img_idx in self.saved_boxes:
                for box_data in self.saved_boxes[img_idx]:
                    # CSV 형식: image_index,x,y,width,height
                    _, x, y, w, h = map(float, box_data.split(','))
                    
                    # 이미지 시작 위치에 따라 x 좌표 조정
                    if img_idx == mid_idx - 1:
                        x += 0  # 첫 번째 이미지
                    elif img_idx == mid_idx:
                        x += self.inspection_config[mid_idx - 1][3]  # 두 번째 이미지
                    else:
                        x += (self.inspection_config[mid_idx - 1][3] + 
                              self.inspection_config[mid_idx][3])  # 세 번째 이미지
                    
                    # 슬라이더 offset 적용
                    x -= start_idx
                    
                    # 화면에 보이는 영역 내의 박스만 표시
                    if -w <= x <= self.viewer_width:
                        # 박스 생성 및 추가
                        box = self.custom_view.create_box(x, y, w, h)
                        self.custom_view.box_items.append(box)
                        self.scene.addItem(box)

    def load_saved_boxes(self, file_path="boxes.csv"):
        """저장된 박스 데이터를 메모리에 로드"""
        try:
            with open(file_path, "r") as f:
                next(f)  # 헤더 건너뛰기
                for line in f:
                    image_index = int(line.split(",")[0])
                    self.saved_boxes.setdefault(image_index, []).append(line.strip())
            print(f"[📂 박스 데이터 로드 완료] {len(self.saved_boxes)} 개의 이미지에 대한 박스 데이터")
        except FileNotFoundError:
            print("[⚠️] 저장된 박스 데이터 파일이 없습니다.")

    def setup_ui_logic(self):
        # 슬라이더 설정
        self.ui.left.setMinimum(-300)
        self.ui.left.setMaximum(300)
        self.ui.left.setValue(0)

        self.ui.right.setMinimum(-7)
        self.ui.right.setMaximum(7)
        self.ui.right.setValue(0)

        self.ui.left.valueChanged.connect(lambda v: self.update_slider("rpm", v))
        self.ui.left.sliderReleased.connect(lambda: self.reset_slider("left", "rpm"))

        self.ui.right.valueChanged.connect(lambda v: self.update_slider("zoom", v))
        self.ui.right.sliderReleased.connect(lambda: self.reset_slider("right", "zoom"))

        # 일반 버튼 토글 처리
        self.ui.auto_drive_button.clicked.connect(lambda: self.toggle_button("auto_drive_button"))
        self.ui.alarm_button.clicked.connect(lambda: self.toggle_button("alarm_button"))
        self.ui.return_button.clicked.connect(lambda: self.toggle_button("return_button"))
        self.ui.setting_button.clicked.connect(lambda: self.toggle_button("setting_button"))

        # 방향 툴버튼 press/release 처리
        self.ui.up_button.pressed.connect(lambda: self.update_button_state("up_button", "pressed"))
        self.ui.up_button.released.connect(lambda: self.update_button_state("up_button", "released"))

        self.ui.down_button.pressed.connect(lambda: self.update_button_state("down_button", "pressed"))
        self.ui.down_button.released.connect(lambda: self.update_button_state("down_button", "released"))

        self.ui.left_button.pressed.connect(lambda: self.update_button_state("left_button", "pressed"))
        self.ui.left_button.released.connect(lambda: self.update_button_state("left_button", "released"))

        self.ui.right_button.pressed.connect(lambda: self.update_button_state("right_button", "pressed"))
        self.ui.right_button.released.connect(lambda: self.update_button_state("right_button", "released"))

        # 총 픽셀 길이 계산
        total_pixels = sum([row[3] for row in self.inspection_config])  # length_pix 필드의 합

        # 수평 스크롤바 설정
        scrollbar = self.ui.horizontalScrollBar
        scrollbar.setMinimum(0)
        scrollbar.setMaximum(total_pixels)

        # 스크롤 이벤트 연결
        scrollbar.valueChanged.connect(self.on_scroll_changed)

    def on_scroll_changed(self, value):
        print(f"[📍 스크롤 위치] {int(value)} px")

        # 화면 크기 기준으로 확대 비율과 뷰어 폭 계산
        _height = 4096
        self.gain = _height / self.ui.LineCamera.height()
        self.viewer_width = int(self.gain * self.ui.LineCamera.width())

        # 현재 중심 이미지 인덱스
        mid_idx = self.image_data[2]
        
        # 중심 이미지의 시작 위치와 길이 → 중심 offset 계산
        mid_image_start = self.inspection_config[mid_idx][4]
        mid_image_length = self.inspection_config[mid_idx][3]
        mid_image_center_offset = mid_image_start + (mid_image_length // 2)

        # 현재 스크롤 위치가 중심 offset을 넘은 경우 → 이미지 전환
        if value >= mid_image_center_offset:
            next_mid_idx = mid_idx + 1
            if next_mid_idx < len(self.inspection_config) - 1 and next_mid_idx != mid_idx:
                print(f"[🔁 새 병합 조건 충족] mid_idx {mid_idx} → {next_mid_idx}")
                self.update_image_data(next_mid_idx)
                self.shift_images('right')
                return

        # 왼쪽으로 이동 시
        elif value < mid_image_start:
            prev_mid_idx = mid_idx - 1
            if prev_mid_idx > 0 and prev_mid_idx != mid_idx:
                print(f"[🔁 병합 → 왼쪽] mid_idx {mid_idx} → {prev_mid_idx}")
                self.update_image_data(prev_mid_idx)
                self.shift_images('left')
                return

        start_idx = int(value) - self.image_data[0]

        # 병합 완료 상태라면 슬라이스를 화면에 표시
        if self.image_data[3]:
            self.display_pixmap_slice(self.image_data[4], start_idx, self.viewer_width)
            # 🔁 박스는 이 뷰 전용이므로 삭제
            for box in self.custom_view.box_items:
                self.scene.removeItem(box)
            self.custom_view.box_items.clear()
            
            # 현재 보이는 이미지에 해당하는 박스들을 불러와서 표시
            self.load_boxes_for_current_view(start_idx)
            
            print("######")
            print(start_idx)
            print(self.image_data)

    def update_slider(self, key, value):
        self.control_state[key] = value
        print(f"[🎚️ 슬라이더] {key} → {value}")

    def reset_slider(self, slider_name, key):
        slider = getattr(self.ui, slider_name)
        slider.setValue(0)
        self.control_state[key] = 0
        print(f"[🔄 복귀] {slider_name} 슬라이더 → 0 초기화")

    def toggle_button(self, key):
        current = self.control_state[key]
        new_state = "on" if current == "off" else "off"
        self.control_state[key] = new_state
        print(f"[🟢 버튼 토글] {key} → {new_state}")

    def update_button_state(self, key, state):
        self.control_state[key] = state
        print(f"[🔘 방향 버튼] {key} → {state}")

    def save_boxes_for_image(self, file_path="boxes.csv"):
        print("[💾 박스 저장]")

        # 병합 이미지에서 세 이미지의 시작 위치 계산
        mid_idx = self.image_data[2]
        img1_start = 0
        img1_len = self.inspection_config[mid_idx - 1][3]
        img2_start = img1_len
        img2_len = self.inspection_config[mid_idx][3]
        img3_start = img2_start + img2_len
        img3_len = self.inspection_config[mid_idx + 1][3]

        image_ranges = [
            (mid_idx - 1, img1_start, img1_len),
            (mid_idx,     img2_start, img2_len),
            (mid_idx + 1, img3_start, img3_len),
        ]

        rows = []

        for i, box in enumerate(self.custom_view.box_items):
            rect = box.rect()
            x, y, w, h = rect.x(), rect.y(), rect.width(), rect.height()

            # 현재 슬라이더 위치를 고려한 절대 좌표 계산
            absolute_x = x + self.ui.horizontalScrollBar.value()

            # 어느 이미지에 해당되는지 판별
            image_index = None
            local_x = None
            for idx, start, length in image_ranges:
                if start <= absolute_x < start + length:
                    image_index = idx
                    local_x = absolute_x - start  # 상대 좌표로 변환
                    break

            if image_index is not None:
                rows.append(f"{image_index},{local_x:.1f},{y:.1f},{w:.1f},{h:.1f}")
                print(f"  Box {i+1} → image {image_index} / local_x={local_x:.1f}, y={y:.1f}, w={w:.1f}, h={h:.1f}")
            else:
                print(f"  Box {i+1} → 이미지 매칭 실패")

        # CSV 저장
        with open(file_path, "w") as f:
            f.write("image_index,x,y,width,height\n")
            for row in rows:
                f.write(row + "\n")

        # CSV 저장 후 → 메모리에도 누적 저장
        for row in rows:
            image_index = int(row.split(",")[0])
            self.saved_boxes.setdefault(image_index, []).append(row)

        print(f"[📂 저장 완료] {file_path}")

    def generate_green_bar_ratio(ratio: float, width=3200, height=30) -> QtGui.QPixmap:
        """
        주어진 비율(ratio)에 따라 녹색 바를 채운 QPixmap 생성
        :param ratio: 0.0 ~ 1.0 범위의 float
        :param width: 전체 바 폭
        :param height: 바 높이
        :return: QPixmap
        """
        ratio = max(0.0, min(1.0, ratio))  # ✅ 안전한 범위로 클램핑

        image = QtGui.QImage(width, height, QtGui.QImage.Format_ARGB32)
        image.fill(QtCore.Qt.transparent)

        painter = QtGui.QPainter(image)
        painter.setBrush(QtGui.QColor(0, 255, 0, 160))  # 반투명 녹색
        painter.setPen(QtCore.Qt.NoPen)

        # 그리기
        filled_width = int(width * ratio)
        painter.drawRect(0, 0, filled_width, height)

        painter.end()
        return QtGui.QPixmap.fromImage(image)

    def generate_red_lines_image(values: list, width=3200, height=30) -> QtGui.QPixmap:
        image = QtGui.QImage(width, height, QtGui.QImage.Format_ARGB32)
        image.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(image)
        pen = QtGui.QPen(QtGui.QColor(255, 0, 0, 180))
        pen.setWidth(4)
        painter.setPen(pen)
        for v in values:
            if 0.0 <= v <= 1.0:
                x = int(v * width)
                painter.drawLine(x, 0, x, height)
        painter.end()
        return QtGui.QPixmap.fromImage(image)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TrackBotApp()

    window.show()
    sys.exit(app.exec_())
