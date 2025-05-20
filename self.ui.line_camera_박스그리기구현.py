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
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QGraphicsView
from box_drawing_view import BoxDrawingView

import image_processing as ip


## Available platform plugins are: xcb, eglfs, linuxfb, 
# minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx.
import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"


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

        # self.scene = QGraphicsScene()
        # self.image_item = QGraphicsPixmapItem()
        # self.scene.addItem(self.image_item)

        # self.ui.LineCamera.setScene(self.scene)
        # self.ui.LineCamera.setRenderHint(QPainter.Antialiasing)
        # self.ui.LineCamera.setDragMode(QGraphicsView.ScrollHandDrag) 
        
        # ✅ 기존 LineCamera → BoxDrawingView로 교체
        self.box_view = BoxDrawingView()
        self.box_scene = QGraphicsScene()
        self.box_view.setScene(self.box_scene)
        self.box_view.setRenderHint(QPainter.Antialiasing)
        self.box_view.setDragMode(QGraphicsView.ScrollHandDrag)

        self.image_item = QGraphicsPixmapItem()
        self.box_scene.addItem(self.image_item)

        # QSplitter 기반으로 교체
        splitter = self.ui.LineCamera.parent()  # QSplitter
        idx = splitter.indexOf(self.ui.LineCamera)
        splitter.insertWidget(idx, self.box_view)
        splitter.widget(idx + 1).deleteLater()  # 기존 LineCamera 제거
        self.ui.LineCamera = self.box_view  # 기존 코드와 호환되도록 유지
        self.image_queue = queue.Queue()

        # 주기적으로 큐에서 이미지 확인
        self.image_check_timer = QTimer()
        self.image_check_timer.timeout.connect(self.check_image_queue)
        self.image_check_timer.start(50)  # 50ms마다 체크
        
        self.merged_image = None
        self.merged_done = False

        # 이미지 속성 초기화
        for i in range(3):  # 3개의 버퍼
            for j in range(3):  # 각 버퍼당 3개의 이미지
                setattr(self, f"image{j+1}_{i}", [None, False])

        # 병합 버퍼 초기화 (self.image_data 형식으로 통일)
        # [시작 offset, 전체길이, 중간 이미지 인덱스, 사용가능여부, Numpy array]
        self.merged_buffers = [
            [0, 0, 0, False, None],  # 이전 버퍼 (0,1,2)
            [0, 0, 0, False, None],  # 현재 버퍼 (1,2,3)
            [0, 0, 0, False, None]   # 다음 버퍼 (2,3,4)
        ]
        self.current_buffer_index = 1  # 가운데 버퍼가 현재
        self.middle_idx = 1            # 현재 기준 인덱스

        # 현재 메모리에 있는 이미지 정보
        # 시작 offset, 전체길이, 중간 이미지 인덱스, 사용가능여부, Numpy array
        self.image_data = [0,0,0,False,None] #1,2,3 3개를 가지고 있기
        # 현재 보고있는 이미지 데이터, 다음 이미지 데이터, 이전 이미지 데이터
        self.current_image_info = ["image_data2", "image_data1", "image_data3"] 

        self.control_state = {
            "rpm": 0,
            "zoom": 0,
            "auto_drive_button": "off",
            "alarm_button": "off",
            "return_button": "off",
            "setting_button": "off",
            "up_button": "released",
            "down_button": "released",
            "left_button": "released",
            "right_button": "released",
        }

        # 🔌 PickleSender 시작
        self.pickle_sender = PickleSender(
            host="ap.loclx.io",
            port=12000,
            get_state_callback=lambda: self.control_state
        )
        threading.Thread(target=self.pickle_sender.run, daemon=True).start()

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

        # screen width pix gain
        _height = 4096
        self.gain = _height/self.ui.LineCamera.height()
        self.viewer_width = int(self.gain * self.ui.LineCamera.width())

        self.setup_ui_logic()

        # 전환 관련 변수 추가
        self.last_transition_time = 0
        self.transition_cooldown = 0.5  # 0.5초 쿨다운
        self.scroll_direction = 0  # -1: 왼쪽, 0: 정지, 1: 오른쪽
        self.last_scroll_value = 0
        self.last_transition_idx = None

        # 시작 offset, 전체길이, 중간 이미지 인덱스, 사용가능여부, Numpy array
        mdx = 1 # 중간 이미지 인덱스

        # 초기 버퍼 설정 # data : image_name,image_path,distance_m,length_pix,total_length
        self.merged_buffers[0][3] = False  # 이전 버퍼는 처음에 없음
        self.merged_buffers[1][0] = inspection_config[mdx-1][4]  # 시작 offset
        self.merged_buffers[1][1] = sum(inspection_config[i][3] for i in range(mdx-1, mdx+2))  # 전체 길이
        self.merged_buffers[1][2] = mdx  # 중간 이미지 인덱스
        self.merged_buffers[2][0] = inspection_config[mdx][4]  # 시작 offset
        self.merged_buffers[2][1] = sum(inspection_config[i][3] for i in range(mdx, mdx+3))  # 전체 길이
        self.merged_buffers[2][2] = mdx + 1  # 중간 이미지 인덱스

        # 현재 버퍼(1)와 다음 버퍼(2) 로드
        self.load_requested_image(mdx-1, target_attr="image1_1")
        self.load_requested_image(mdx, target_attr="image2_1")
        self.load_requested_image(mdx+1, target_attr="image3_1")
        
        self.load_requested_image(mdx, target_attr="image1_2")
        self.load_requested_image(mdx+1, target_attr="image2_2")
        self.load_requested_image(mdx+2, target_attr="image3_2")
        
        # 병합 감시 스레드 시작
        self.start_merge_monitor_thread()

        #################################################################################




    ######### multi thread ###########
    def load_requested_image(self, idx, target_attr):
        image_path = self.inspection_config[idx][1]
        loader = ip.ImageLoadRequestThread(idx, image_path, lambda i, img: self.handle_loaded_image(i, img, target_attr))
        loader.start()

    def check_image_queue(self):
        try:
            image_np = self.image_queue.get_nowait()
            self.handle_loaded_image(image_np)
        except queue.Empty:
            pass


    def handle_loaded_image(self, idx, image_np, target_attr):
        # 기존: [None, False] → 변경: [image_np, True]
        setattr(self, target_attr, [image_np, True])
        print(f"[✅ 로드 완료] {idx}번째 → self.{target_attr} 에 저장됨 + 상태 변경됨")


    def display_pixmap_slice(self, pixmap, start_x, width):
        height = pixmap.height()
        sub_pixmap = pixmap.copy(start_x, 0, width, height)

        self.image_item.setPixmap(sub_pixmap)
        self.box_scene.setSceneRect(0, 0, sub_pixmap.width(), height)
        self.ui.LineCamera.fitInView(self.image_item, Qt.IgnoreAspectRatio)


    def start_merge_monitor_thread(self):
        def monitor_loop():
            while not self.merged_done:
                # 현재 버퍼의 이미지들이 모두 로드되었는지 확인
                current_buffer = self.merged_buffers[self.current_buffer_index]
                if current_buffer[3]:  # 이미 병합된 버퍼인 경우
                    print("[🔁] 버퍼 이미지 로드 완료. 병합 시작.")
                    try:
                        self.merged_image = np.concatenate(
                            [getattr(self, f"image{i+1}_{j}")[0] for i in range(3) for j in range(3)],
                            axis=1
                        )
                        self.merged_done = True
                        print("[✅] 이미지 병합 완료.")

                        # 👉 NumPy → QImage → QPixmap
                        height, width, ch = self.merged_image.shape
                        bytes_per_line = ch * width
                        qimg = QImage(
                            self.merged_image.tobytes(), width, height, bytes_per_line, QImage.Format_RGB888
                        ).rgbSwapped()
                        self.cached_pixmap = QPixmap.fromImage(qimg)

                        # 이미지 상태 플래그
                        self.image_data = [
                            self.inspection_config[self.middle_idx-1][4],
                            self.inspection_config[self.middle_idx][3] + self.inspection_config[self.middle_idx+1][3],
                            self.middle_idx,
                            True,
                            self.cached_pixmap
                        ]

                    except Exception as e:
                        print(f"[❌] 병합 실패: {e}")
                        break
                time.sleep(0.1)
        threading.Thread(target=monitor_loop, daemon=True).start()


    ######### multi thread ###########


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

        # 초기 버퍼 3개 준비
        self.build_merged_buffer(self.middle_idx - 1, 0)
        self.build_merged_buffer(self.middle_idx, 1)
        self.build_merged_buffer(self.middle_idx + 1, 2)
        self.last_transition_idx = self.middle_idx

    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
  
    def on_scroll_changed(self, value):
        print(f"[📍 스크롤 위치] {int(value)} px")

        # 스크롤 방향 계산
        if value > self.last_scroll_value:
            self.scroll_direction = 1
        elif value < self.last_scroll_value:
            self.scroll_direction = -1
        self.last_scroll_value = value

        # 쿨다운 체크
        current_time = time.time()
        if current_time - self.last_transition_time < self.transition_cooldown:
            return

        _height = 4096
        self.gain = _height / self.ui.LineCamera.height()
        self.viewer_width = int(self.gain * self.ui.LineCamera.width())

        mid_idx = self.image_data[2]
        mid_image_start = self.inspection_config[mid_idx][4]
        mid_image_length = self.inspection_config[mid_idx][3]
        mid_image_center_offset = mid_image_start + (mid_image_length // 2)

        # 오른쪽으로 이동 (다음 이미지 중심 넘음)
        if (value >= mid_image_center_offset -1000 and 
            self.scroll_direction == 1 and 
            self.last_transition_idx != mid_idx + 1):
            
            next_mid_idx = mid_idx + 1
            if next_mid_idx + 1 < len(self.inspection_config):  # 마지막 체크
                print(f"[🔁 전환: {mid_idx} → {next_mid_idx}]")
                self.last_transition_time = current_time
                self.last_transition_idx = next_mid_idx

                # 다음 이미지 세트의 시작 위치 계산
                next_start_offset = self.inspection_config[next_mid_idx - 1][4]
                relative_offset = value - mid_image_start
                new_value = next_start_offset + relative_offset

                # 버퍼 순환
                self.transition_buffers(1)  # 오른쪽 이동
                
                def on_ready():
                    self.ui.horizontalScrollBar.blockSignals(True)
                    self.ui.horizontalScrollBar.setValue(int(new_value))
                    self.ui.horizontalScrollBar.blockSignals(False)
                    if self.merged_buffers[1][3]:  # valid 체크
                        start_x = new_value - self.merged_buffers[1][0]  # start_offset
                        self.display_pixmap_slice(self.merged_buffers[1][4], start_x, self.viewer_width)

                # 새로운 다음 버퍼 준비
                self.build_merged_buffer(next_mid_idx, 2, on_ready_callback=on_ready)
                return

        # 왼쪽으로 이동 (이전 이미지 중심 넘음)
        elif (value < mid_image_start + 1000 and 
              self.scroll_direction == -1 and 
              self.last_transition_idx != mid_idx - 1):
            
            prev_mid_idx = mid_idx - 1
            if prev_mid_idx - 1 >= 0:  # 첫 이미지 체크
                print(f"[🔁 전환: {mid_idx} → {prev_mid_idx}]")
                self.last_transition_time = current_time
                self.last_transition_idx = prev_mid_idx

                # 이전 이미지 세트의 시작 위치 계산
                prev_start_offset = self.inspection_config[prev_mid_idx - 1][4]
                relative_offset = value - mid_image_start
                new_value = prev_start_offset + relative_offset

                # 버퍼 순환
                self.transition_buffers(-1)  # 왼쪽 이동

                def on_ready():
                    self.ui.horizontalScrollBar.blockSignals(True)
                    self.ui.horizontalScrollBar.setValue(int(new_value))
                    self.ui.horizontalScrollBar.blockSignals(False)
                    if self.merged_buffers[1][3]:  # valid 체크
                        start_x = new_value - self.merged_buffers[1][0]  # start_offset
                        self.display_pixmap_slice(self.merged_buffers[1][4], start_x, self.viewer_width)

                # 새로운 이전 버퍼 준비
                self.build_merged_buffer(prev_mid_idx - 1, 0, on_ready_callback=on_ready)
                return

        # 현재 버퍼 표시
        if self.image_data[3] and self.image_data[4] is not None:
            start_x = value - self.image_data[0]
            if 0 <= start_x <= self.image_data[4].width() - self.viewer_width:
                self.display_pixmap_slice(self.image_data[4], start_x, self.viewer_width)
        
        self.print_buffer_status()


    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#

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

    def transition_buffers(self, direction):
        if direction == 1:  # 오른쪽 이동
            # 현재 버퍼를 이전 버퍼로
            self.update_buffer_status(0, True)
            self.merged_buffers[0] = self.merged_buffers[1].copy()
            
            # 다음 버퍼를 현재 버퍼로
            self.merged_buffers[1] = self.merged_buffers[2].copy()
            
            # 새로운 다음 버퍼 준비
            next_mid_idx = self.merged_buffers[1][2] + 1  # 현재 중간 인덱스 + 1
            if next_mid_idx + 1 < len(self.inspection_config):
                self.update_buffer_status(2, False)
                self.build_merged_buffer(next_mid_idx - 1, 2)  # 중간 이미지의 이전 이미지부터 시작
                self.image_data = self.merged_buffers[1].copy()  # 🔧 추가됨
                self.middle_idx = self.merged_buffers[1][2]       # 🔧 추가됨
        else:  # 왼쪽 이동
            # 현재 버퍼를 다음 버퍼로
            self.update_buffer_status(2, True)
            self.merged_buffers[2] = self.merged_buffers[1].copy()
            
            # 이전 버퍼를 현재 버퍼로
            self.merged_buffers[1] = self.merged_buffers[0].copy()
            
            # 새로운 이전 버퍼 준비
            prev_mid_idx = self.merged_buffers[1][2] - 1  # 현재 중간 인덱스 - 1
            if prev_mid_idx - 1 >= 0:
                self.update_buffer_status(0, False)
                self.build_merged_buffer(prev_mid_idx - 1, 0)
            self.image_data = self.merged_buffers[1].copy()  # 🔧 추가됨
            self.middle_idx = self.merged_buffers[1][2]       # 🔧 추가됨

    def update_buffer_status(self, buffer_slot, is_valid=True):
        if is_valid:
            self.merged_buffers[buffer_slot][3] = True
            print(f"[✅ 버퍼 {buffer_slot} 활성화]")
        else:
            self.merged_buffers[buffer_slot][3] = False
            self.merged_buffers[buffer_slot][4] = None  # 메모리 정리
            print(f"[🔄 버퍼 {buffer_slot} 비활성화]")

    def build_merged_buffer(self, start_idx, buffer_slot, on_ready_callback=None):
        try:
            if start_idx + 2 >= len(self.inspection_config):
                print(f"[⚠️ 버퍼 {buffer_slot}] 인덱스 초과로 병합 생략")
                return

            # 버퍼 초기화
            self.merged_buffers[buffer_slot] = [0, 0, 0, False, None]
            
            # 이미지 로딩 순서 보장
            for i in range(3):
                self.load_requested_image(start_idx + i, f"image{i+1}_{buffer_slot}")

            while not all([
                getattr(self, f"image{i+1}_{buffer_slot}")[1]
                for i in range(3)
            ]):
                time.sleep(0.1)

            imgs = [
                getattr(self, f"image{i+1}_{buffer_slot}")[0]
                for i in range(3)
            ]
            merged = np.concatenate(imgs, axis=1)

            h, w, ch = merged.shape
            qimg = QImage(merged.tobytes(), w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
            qpixmap = QPixmap.fromImage(qimg)

            # 버퍼 정보 업데이트
            start_offset = self.inspection_config[start_idx][4]
            total_length = sum(self.inspection_config[i][3] for i in range(start_idx, start_idx+3))
            mid_idx = start_idx + 1

            # 버퍼 정보 설정
            self.merged_buffers[buffer_slot] = [
                start_offset,    # 시작 offset (중간 이미지 기준)
                total_length,    # 전체 길이
                mid_idx,        # 중간 이미지 인덱스
                True,           # 사용 가능
                qpixmap         # 이미지 데이터
            ]

            if buffer_slot == self.current_buffer_index:
                self.image_data = self.merged_buffers[buffer_slot].copy()

            print(f"[✅ 병합 완료] 버퍼 {buffer_slot}")
            print(f"    - 시작 인덱스: {start_idx}")
            print(f"    - 중간 인덱스: {mid_idx}")
            print(f"    - 범위: {start_offset}~{start_offset+total_length}")
            print(f"    - 크기: {w}x{h}")

            if on_ready_callback:
                on_ready_callback()

        except Exception as e:
            print(f"[❌ 버퍼 병합 실패] {e}")

    def print_buffer_status(self):
        print("\n🧊 [버퍼 상태 상세]")
        for idx, buffer in enumerate(self.merged_buffers):
            valid = buffer[3]
            start_offset = buffer[0]
            mid_idx = buffer[2]
            print(f"  버퍼 {idx}: {'✅' if valid else '❌'}")
            print(f"    - 중간 이미지: {mid_idx}")
            print(f"    - 범위: {start_offset}~{start_offset+buffer[1]}")
            print(f"    - 크기: {buffer[4].width() if buffer[4] else 0}x{buffer[4].height() if buffer[4] else 0}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TrackBotApp()

    window.show()
    sys.exit(app.exec_())
