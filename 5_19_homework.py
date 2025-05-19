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

        self.scene = QGraphicsScene()
        self.image_item = QGraphicsPixmapItem()
        self.scene.addItem(self.image_item)

        self.ui.LineCamera.setScene(self.scene)
        self.ui.LineCamera.setRenderHint(QPainter.Antialiasing)
        self.ui.LineCamera.setDragMode(QGraphicsView.ScrollHandDrag) 

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

        # 병합 버퍼 초기화
        self.merged_buffers = [
            {"images": [None, None, None], "pixmap": None, "valid": False, "start_idx": 0},
            {"images": [None, None, None], "pixmap": None, "valid": False, "start_idx": 0},
            {"images": [None, None, None], "pixmap": None, "valid": False, "start_idx": 0},
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

        # 시작 offset, 전체길이, 중간 이미지 인덱스, 사용가능여부, Numpy array
        mdx = 1 # 중간 이미지 인덱스

        self.image_data[0] = inspection_config[mdx-1][4] # 첫번째 사진의 total_length
        self.image_data[1] = inspection_config[mdx-1][3] + inspection_config[mdx][3] + inspection_config[mdx+1][3]
        self.image_data[2] = mdx

        self.load_requested_image(mdx-1, target_attr="image1")
        self.load_requested_image(mdx, target_attr="image2")
        self.load_requested_image(mdx+1, target_attr="image3")

        
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
        self.scene.setSceneRect(0, 0, sub_pixmap.width(), height)
        self.ui.LineCamera.fitInView(self.image_item, Qt.IgnoreAspectRatio)


    def start_merge_monitor_thread(self):
        def monitor_loop():
            while not self.merged_done:
                # 현재 버퍼의 이미지들이 모두 로드되었는지 확인
                current_buffer = self.merged_buffers[self.current_buffer_index]
                if current_buffer["valid"]:  # 이미 병합된 버퍼인 경우
                    print("[🔁] 버퍼 이미지 로드 완료. 병합 시작.")
                    try:
                        self.merged_image = np.concatenate(
                            current_buffer["images"],
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
                        self.image_data[4] = self.cached_pixmap
                        self.image_data[3] = True

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


    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
    def on_scroll_changed(self, value):
        print(f"[📍 스크롤 위치] {int(value)} px")

        # 화면 크기 기준으로 확대 비율과 뷰어 폭 계산
        _height = 4096
        self.gain = _height / self.ui.LineCamera.height()
        self.viewer_width = int(self.gain * self.ui.LineCamera.width())

        # 현재 버퍼 정보
        buf = self.merged_buffers[self.current_buffer_index]
        if not buf["valid"]:
            return

        # 현재 중심 이미지의 시작 위치와 길이 → 중심 offset 계산
        mid_image_start = self.inspection_config[self.middle_idx][4]
        mid_image_length = self.inspection_config[self.middle_idx][3]
        mid_image_center_offset = mid_image_start + (mid_image_length // 2)

        # 현재 스크롤 위치가 중심 offset을 넘은 경우 → 이미지 전환
        if value >= mid_image_center_offset:
            next_mid_idx = self.middle_idx + 1

            # 끝에 도달하지 않았는지 확인
            if next_mid_idx < len(self.inspection_config) - 1:
                print(f"[🔁 새 병합 조건 충족] mid_idx {self.middle_idx} → {next_mid_idx}")

                # 다음 버퍼로 전환
                self.middle_idx = next_mid_idx
                self.current_buffer_index = (self.current_buffer_index + 1) % 3
                
                # 새로운 버퍼 준비
                self.build_merged_buffer(next_mid_idx, (self.current_buffer_index + 1) % 3)
                return  # 병합 중이므로 표시 생략

        # 현재 버퍼에서 슬라이싱하여 표시
        if buf["valid"]:
            start_x = value - buf["start_idx"]
            self.display_pixmap_slice(buf["pixmap"], start_x, self.viewer_width)


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

    def build_merged_buffer(self, start_idx, buffer_slot):
        try:
            # 이미지 로드 요청
            self.load_requested_image(start_idx, target_attr=f"image1_{buffer_slot}")
            self.load_requested_image(start_idx + 1, target_attr=f"image2_{buffer_slot}")
            self.load_requested_image(start_idx + 2, target_attr=f"image3_{buffer_slot}")

            # 이미지 로드 완료 대기
            while not all([
                getattr(self, f"image1_{buffer_slot}")[1],
                getattr(self, f"image2_{buffer_slot}")[1],
                getattr(self, f"image3_{buffer_slot}")[1]
            ]):
                time.sleep(0.1)

            # 이미지 병합
            imgs = [
                getattr(self, f"image1_{buffer_slot}")[0],
                getattr(self, f"image2_{buffer_slot}")[0],
                getattr(self, f"image3_{buffer_slot}")[0]
            ]
            merged = np.concatenate(imgs, axis=1)
            
            # QPixmap 변환
            h, w, ch = merged.shape
            qimg = QImage(merged.tobytes(), w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
            qpixmap = QPixmap.fromImage(qimg)

            self.merged_buffers[buffer_slot] = {
                "images": imgs,
                "pixmap": qpixmap,
                "valid": True,
                "start_idx": self.inspection_config[start_idx][4]
            }

            print(f"[✅ 버퍼 {buffer_slot}] 병합 완료: {start_idx}~{start_idx+2}")

        except Exception as e:
            print(f"[❌ 버퍼 병합 실패] {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TrackBotApp()

    window.show()
    sys.exit(app.exec_())
