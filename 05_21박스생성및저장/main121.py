import sys
import socket
import struct
import pickle
import time
import threading
import queue
import numpy as np
import sip
import json

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from UI.trackbot_gui import Ui_MainWindow  # .ui를 pyuic5로 변환한 파일
from PyQt5.QtGui import QImage, QPixmap, QPainter
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QGraphicsView
from box_drawing_view import BoxDrawingView

from UI.alarm_popup import AlarmPopupDialog
from UI.setting_popup import SettingPopupDialog
from UI.camera_popup import CameraPopupDialog

from tcp_image_receiver import TcpImageReceiver
import image_processing as ip
import cv2

## Available platform plugins are: xcb, eglfs, linuxfb, 
# minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx.
import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"


import os

image_folder = "inspection_result/image"
os.makedirs(image_folder, exist_ok=True)


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
        ###############################################################################
        # 🔌 PickleSender 시작
        self.pickle_sender = PickleSender(
            host="ap.loclx.io",
            port=12000,
            get_state_callback=lambda: self.control_state
        )
        threading.Thread(target=self.pickle_sender.run, daemon=True).start()

        ###############################################################################
        
        # TcpImageReciver 선언
        self.image_save_index = 0
        self.image_receiver = TcpImageReceiver(
            host="ap.loclx.io",
            port=12100,
            on_image_received=self.handle_received_image
        )
        self.image_receiver.start()

        ###############################################################################
        # 설정파일 불러오기
        inspection_config = ip.load_inspection_config("Config/inspection_config.txt")
        print(inspection_config)

        # 픽셀거리정보 생성
        tot_length = 0
        for idx, data in enumerate(inspection_config):
            # data : image_name,image_path,distance_m,length_pix,total_length
            _length = ip.calculate_required_pixels(data[2],1250)
            inspection_config[idx][3] = _length
            inspection_config[idx][4] = tot_length
            tot_length += _length
            # 수신 여부 추가 (초기값: False)
            inspection_config[idx].append(False)

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

        # 박스 생성, 삭제 시그널 연결 (절대 좌표 기반)
        self.box_view.boxCreated.connect(self.handle_box_created)
        self.box_view.boxDeleted.connect(self.handle_box_deleted)
        # self.box_view.boxUpdated.connect(self.save_all_current_boxes) # 이제 박스 생성/삭제 시 개별적으로 저장

        # 전환 관련 변수 추가
        self.last_transition_time = 0
        self.transition_cooldown = 0.5  # 0.5초 쿨다운
        self.scroll_direction = 0  # -1: 왼쪽, 0: 정지, 1: 오른쪽
        self.last_scroll_value = 0
        self.last_transition_idx = self.middle_idx # 초기값 설정


        # 병합 감시 스레드 시작
        self.start_merge_monitor_thread()

        # 초기 박스 로드
        self.box_view.load_boxes_absolute("GUI_boxes/boxes.json", self.image_data[0], self.viewer_width)

        #################################################################################




    ######### multi thread ###########
    def load_requested_image(self, idx, target_attr):
        image_path = self.inspection_config[idx][1]
        loader = ip.ImageLoadRequestThread(idx, image_path, lambda i, img: self.handle_loaded_image(i, img, target_attr))
        loader.start()

    def check_image_queue(self):
        try:
            image_np = self.image_queue.get_nowait() # 이 부분은 다른 이미지 로딩 로직과 겹치는지 확인 필요
            # self.handle_loaded_image(image_np) # 이 함수는 이제 스레드 콜백으로 사용
        except queue.Empty:
            pass


    def handle_loaded_image(self, idx, image_np, target_attr):
        # 기존: [None, False] → 변경: [image_np, True]
        setattr(self, target_attr, [image_np, True])
        # print(f"[✅ 로드 완료] {idx}번째 → self.{target_attr} 에 저장됨 + 상태 변경됨")

        # 버퍼 병합 및 업데이트 로직은 start_merge_monitor_thread 에서 처리

    def display_pixmap_slice(self, pixmap, start_x, width):
        height = pixmap.height()
        sub_pixmap = pixmap.copy(start_x, 0, width, height)

        self.image_item.setPixmap(sub_pixmap)
        self.box_scene.setSceneRect(0, 0, sub_pixmap.width(), height)
        self.ui.LineCamera.fitInView(self.image_item, Qt.IgnoreAspectRatio)


    def start_merge_monitor_thread(self):
        def monitor_loop():
            while True:
                # 현재 버퍼 인덱스의 이미지들이 모두 로드되었는지 확인
                current_buffer_slot = self.current_buffer_index
                buffer_ready = True
                for i in range(3):
                    attr_name = f"image{i+1}_{current_buffer_slot}"
                    if not hasattr(self, attr_name) or getattr(self, attr_name)[0] is None:
                         buffer_ready = False
                         break

                if buffer_ready and not self.merged_buffers[current_buffer_slot][3]: # 로드 완료 & 아직 병합 안됨
                    # ① 실제로 존재하는 인덱스만 추려낸다.
                    start_idx = self.merged_buffers[current_buffer_slot][2] -1 # 중간 이미지 인덱스 기준 -1 (3개 이미지 사용)
                    valid_indices = [
                        i for i in range(start_idx, start_idx + 3)
                        if 0 <= i < len(self.inspection_config)
                    ]

                    if not valid_indices:
                        # print(f"[⚠️ 버퍼 {current_buffer_slot}] 유효한 이미지가 없어 병합 생략")
                        time.sleep(0.1) # 너무 빠르게 루프 돌지 않도록 대기
                        continue

                    try:
                        imgs = [
                            getattr(self, f"image{i+1}_{current_buffer_slot}")[0]
                            for i in range(len(valid_indices))
                            if getattr(self, f"image{i+1}_{current_buffer_slot}")[0] is not None # 로드된 이미지 필터링
                        ]
                        
                        if not imgs: # 유효한 로드된 이미지가 없으면 스킵
                             time.sleep(0.1)
                             continue

                        merged = np.concatenate(imgs, axis=1)

                        h, w, ch = merged.shape
                        # QImage 생성 시 bytesPerLine 계산 수정
                        bytes_per_line = ch * w
                        qimg = QImage(merged.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped() #.rgbSwapped() 추가
                        qpixmap = QPixmap.fromImage(qimg)

                        # 버퍼 정보 업데이트
                        # 시작 offset 계산 (첫 번째 유효 이미지의 시작 offset)
                        start_offset = self.inspection_config[valid_indices[0]][4]
                        
                        # 전체 길이 계산 (마지막 유효 이미지의 끝 offset - 첫 번째 유효 이미지의 시작 offset)
                        last_valid_idx = valid_indices[-1]
                        total_length = (self.inspection_config[last_valid_idx][4] + self.inspection_config[last_valid_idx][3]) - start_offset
                        
                        mid_idx = self.merged_buffers[current_buffer_slot][2] # 버퍼의 중간 이미지 인덱스는 그대로 유지

                        # 버퍼 정보 설정
                        self.merged_buffers[current_buffer_slot] = [
                            start_offset,    # 시작 offset 
                            total_length,    # 전체 길이
                            mid_idx,        # 중간 이미지 인덱스
                            True,           # 사용 가능
                            qpixmap         # 이미지 데이터
                        ]

                        if current_buffer_slot == self.current_buffer_index:
                             self.image_data = self.merged_buffers[current_buffer_slot].copy()
                             # 현재 버퍼 로드 완료 후 박스 로드
                             current_scroll_value = self.ui.horizontalScrollBar.value()
                             self.box_view.load_boxes_absolute("GUI_boxes/boxes.json", self.image_data[0], self.viewer_width)
                             self.display_pixmap_slice(self.image_data[4], current_scroll_value - self.image_data[0], self.viewer_width) # 이미지 표시

                        # print(f"[✅ 병합 완료] 버퍼 {current_buffer_slot}")
                        # self.print_buffer_status()

                    except Exception as e:
                        print(f"[❌ 버퍼 병합 실패] {e}")
                        # 실패 시 해당 버퍼를 다시 로드 대기 상태로
                        self.merged_buffers[current_buffer_slot][3] = False 

                time.sleep(0.05) # 짧은 대기 시간

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

        self.ui.auto_drive_button.clicked.connect(self.show_camera_popup)
        self.ui.alarm_button.clicked.connect(self.show_alarm_popup)
        self.ui.return_button.clicked.connect(self.show_camera_popup)  # 예시
        self.ui.setting_button.clicked.connect(self.show_setting_popup)

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
        scrollbar.setMaximum(total_pixels - self.viewer_width)

        # 스크롤 이벤트 연결
        scrollbar.valueChanged.connect(self.on_scroll_changed)

        # 초기 버퍼 3개 준비


        # 버퍼 0: 초기에 비워둠
        self.merged_buffers[0][3] = False

        # 버퍼 1: 이미지 0, 1, 2
        self.build_merged_buffer(self.middle_idx - 1, 1)

        # 버퍼 2: 이미지 1, 2, 3
        self.build_merged_buffer(self.middle_idx, 2)

        #image_data에도 현재 버퍼 정보 저장
        self.image_data = self.merged_buffers[1].copy()
        
        self.last_transition_idx = self.middle_idx

    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
  
    def on_scroll_changed(self, value):
        # print(f"[📍 스크롤 위치] {int(value)} px")

        # 스크롤 방향 계산
        if value > self.last_scroll_value:
            self.scroll_direction = 1
        elif value < self.last_scroll_value:
            self.scroll_direction = -1
        # else:
        #     self.scroll_direction = 0 # 스크롤 멈춤
        self.last_scroll_value = value

        # 쿨다운 체크
        current_time = time.time()
        if current_time - self.last_transition_time < self.transition_cooldown:
             # 쿨다운 중에도 현재 버퍼 이미지는 표시
            if self.image_data[3] and self.image_data[4] is not None:
                 start_x = value - self.image_data[0]
                 # 현재 버퍼 이미지 범위 내에 있는지 확인
                 if 0 <= start_x <= self.image_data[4].width() - self.viewer_width:
                     self.display_pixmap_slice(self.image_data[4], start_x, self.viewer_width)
                     # 스크롤 중에도 박스 로드는 계속 수행 (보이는 범위 갱신) 
                     #!!스크롤 이벤트 발생 시 마다 로드하면 성능 저하 가능!!# 일단 구현해둠
                     self.box_view.load_boxes_absolute("GUI_boxes/boxes.json", self.image_data[0], self.viewer_width) # 🔧 추가됨: 스크롤 중 박스 동기화

            # else:
                 # print("[⚠️ 이미지 준비 안됨]") # 디버깅용
            return # 쿨다운 중에는 버퍼 전환 로직 스킵

        _height = 4096
        self.gain = _height / self.ui.LineCamera.height()
        self.viewer_width = int(self.gain * self.ui.LineCamera.width())

        mid_idx = self.image_data[2] # 현재 버퍼의 중간 이미지 인덱스
        
        # 다음 이미지의 시작 위치 계산 (현재 중간 이미지 기준)
        next_image_start_abs = self.inspection_config[mid_idx][4] if mid_idx < len(self.inspection_config) else float('inf')

        # 이전 이미지의 시작 위치 계산 (현재 중간 이미지 기준)
        prev_image_start_abs = self.inspection_config[mid_idx-2][4] if mid_idx-2 >= 0 else float('-inf')

        # 오른쪽으로 이동 (다음 이미지 시작 위치 넘음) - 이전 중간 이미지 인덱스 기준으로 판단
        # 이전 중간 이미지 인덱스의 시작 절대 좌표를 사용해야 합니다.
        # prev_mid_image_start_abs = self.inspection_config[self.last_transition_idx][4] if self.last_transition_idx is not None and self.last_transition_idx < len(self.inspection_config) else -1

        # self.last_transition_idx가 유효한 인덱스 범위 내에 있고 None이 아닌지 확인
        if (self.last_transition_idx is not None and 
            0 <= self.last_transition_idx < len(self.inspection_config) and
            self.scroll_direction == 1 and 
            value >= self.inspection_config[self.last_transition_idx][4] + self.inspection_config[self.last_transition_idx][3] // 2 and # 이전 중간 이미지 중심을 넘었는지
            self.last_transition_idx + 1 < len(self.inspection_config)): # 다음 중간 이미지가 유효한 범위에 있는지
            
            next_mid_idx = self.last_transition_idx + 1
            print(f"[🔁 전환: {self.last_transition_idx} → {next_mid_idx} (오른쪽)]")
            self.last_transition_time = current_time
            self.last_transition_idx = next_mid_idx

            # 버퍼 순환
            self.transition_buffers(1)  # 오른쪽 이동

            # 스크롤바 위치를 새 버퍼의 상대 위치로 조정
            # 새 버퍼의 시작 offset: self.merged_buffers[1][0]
            # 현재 절대 스크롤 위치: value
            # 새 상대 스크롤 위치: value - self.merged_buffers[1][0]
            # new_relative_scroll_value = value - self.merged_buffers[1][0] # 사용되지 않는 변수

            self.ui.horizontalScrollBar.blockSignals(True)
            # 스크롤바 최대값 고려하여 설정
            max_scroll = self.ui.horizontalScrollBar.maximum()
            # self.ui.horizontalScrollBar.setValue(min(int(value), max_scroll)) # 스크롤 위치는 자동으로 업데이트됨
            self.ui.horizontalScrollBar.blockSignals(False)

            # 새로운 다음 버퍼 준비
            # build_merged_buffer(시작 이미지 인덱스, 버퍼 슬롯 인덱스)
            # 오른쪽 이동했으므로, 새로운 버퍼2는 현재 버퍼1의 다음 세트 이미지를 로드해야 함
            # 현재 버퍼1의 중간 이미지가 next_mid_idx 이므로, 새로운 버퍼2는 next_mid_idx 에서 시작하는 3개 이미지 (next_mid_idx, next_mid_idx+1, next_mid_idx+2)
            if next_mid_idx + 1 < len(self.inspection_config): # 다음 버퍼의 중간 이미지가 유효 범위에 있을 때만 로드
                self.build_merged_buffer(next_mid_idx, 2)

            # 버퍼 전환 완료 후 현재 버퍼의 박스 로드 및 이미지 표시 (비동기 로딩 고려)
            # 이미지 로딩 완료는 start_merge_monitor_thread에서 처리하므로, 여기서 직접 display/load 하지 않음

            return # 오른쪽 이동 처리 완료

        # 왼쪽으로 이동 (이전 이미지 시작 위치 넘음) - 이전 중간 이미지 인덱스 기준으로 판단
        # 현재 중간 이미지 인덱스의 시작 절대 좌표를 사용해야 합니다.
        current_mid_image_start_abs = self.inspection_config[mid_idx][4] if mid_idx < len(self.inspection_config) else float('inf')

        if (self.scroll_direction == -1 and 
            value < current_mid_image_start_abs and 
            self.last_transition_idx is not None and
            self.last_transition_idx - 1 >= 0): # 이전 중간 이미지가 유효한 범위에 있는지

            prev_mid_idx = self.last_transition_idx - 1
            print(f"[🔁 전환: {self.last_transition_idx} → {prev_mid_idx} (왼쪽)]")
            self.last_transition_time = current_time
            self.last_transition_idx = prev_mid_idx

            # 버퍼 순환
            self.transition_buffers(-1)  # 왼쪽 이동

            # 스크롤바 위치를 새 버퍼의 상대 위치로 조정
            # 새 버퍼의 시작 offset: self.merged_buffers[1][0]
            # 현재 절대 스크롤 위치: value
            # 새 상대 스크롤 위치: value - self.merged_buffers[1][0]
            # new_relative_scroll_value = value - self.merged_buffers[1][0] # 사용되지 않는 변수

            self.ui.horizontalScrollBar.blockSignals(True)
            # 스크롤바 최소값 고려하여 설정
            min_scroll = self.ui.horizontalScrollBar.minimum()
            # self.ui.horizontalScrollBar.setValue(max(int(value), min_scroll)) # 스크롤 위치는 자동으로 업데이트됨
            self.ui.horizontalScrollBar.blockSignals(False)

            # 새로운 이전 버퍼 준비
            # build_merged_buffer(시작 이미지 인덱스, 버퍼 슬롯 인덱스)
            # 왼쪽 이동했으므로, 새로운 버퍼0은 현재 버퍼1의 이전 세트 이미지를 로드해야 함
            # 현재 버퍼1의 중간 이미지가 prev_mid_idx 이므로, 새로운 버퍼0은 prev_mid_idx 에서 시작하는 3개 이미지 (prev_mid_idx, prev_mid_idx+1, prev_mid_idx+2)
            if prev_mid_idx -1 >= 0: # 이전 버퍼의 중간 이미지가 유효 범위에 있을 때만 로드
                self.build_merged_buffer(prev_mid_idx -1, 0) # 시작 인덱스는 이전 중간 이미지의 인덱스 -1

            # 버퍼 전환 완료 후 현재 버퍼의 박스 로드 및 이미지 표시 (비동기 로딩 고려)
            # 이미지 로딩 완료는 start_merge_monitor_thread에서 처리하므로, 여기서 직접 display/load 하지 않음

            return # 왼쪽 이동 처리 완료

        # 버퍼 전환이 일어나지 않은 경우 현재 버퍼에서 이미지 슬라이스 표시 및 박스 로드
        if self.image_data[3] and self.image_data[4] is not None:
            start_x = value - self.image_data[0] # 현재 버퍼 시작 오프셋을 기준으로 상대 위치 계산
            # 현재 버퍼 이미지 범위 내에 있는지 확인
            if 0 <= start_x <= self.image_data[4].width() - self.viewer_width:
                self.display_pixmap_slice(self.image_data[4], start_x, self.viewer_width)
                # 스크롤 중에도 박스 로드는 계속 수행 (보이는 범위 갱신)
                #!!스크롤 이벤트 발생 시 마다 로드하면 성능 저하 가능!!# 일단 구현해둠
                self.box_view.load_boxes_absolute("GUI_boxes/boxes.json", self.image_data[0], self.viewer_width) # �� 추가됨: 스크롤 중 박스 동기화
        
        # 스크롤 중 박스 로드는 버퍼 전환 완료 시 또는 별도 타이머로 처리하는 것이 효율적일 수 있습니다.
        # 현재는 버퍼 전환 시 start_merge_monitor_thread에서 처리하도록 되어 있습니다.
        # 쿨다운 중에도 박스 로드가 되도록 on_scroll_changed 시작 부분에 load_boxes_absolute 호출 추가했음.

        # self.print_buffer_status() # 디버깅용

    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#

    # 절대 좌표 기반 박스 핸들링 함수 추가
    def handle_box_created(self, box_data_local):
        """박스 생성 시 호출되어 절대 좌표를 계산하고 파일에 저장"""
        # 마지막에 추가된 박스 아이템 가져오기
        if not self.box_view.box_items:
            return
            
        # 로컬 좌표를 이용해 절대 좌표 계산
        x_absolute = box_data_local["x"] + self.ui.horizontalScrollBar.value()
        y = box_data_local["y"]
        width = box_data_local["width"]
        height = box_data_local["height"]

        # 절대 좌표 데이터 생성
        box_data_absolute = {
           "x_absolute": x_absolute,
           "y": y,
           "width": width,
           "height": height
        }

        # 어느 세그먼트에 속하는지 찾기
        found_segment_id = None
        for idx, config in enumerate(self.inspection_config):
            segment_start = config[4]  # total_length is the start offset
            segment_length = config[3]  # length_pix
            segment_end = segment_start + segment_length
            # 박스의 시작점 또는 끝점이 세그먼트 범위에 들어오는지 확인
            if (segment_start <= x_absolute < segment_end) or (segment_start <= x_absolute + width <= segment_end):
                 # 편의상 박스의 시작점이 속한 세그먼트로 결정
                 if segment_start <= x_absolute < segment_end:
                      found_segment_id = idx
                      break

        if found_segment_id is None:
            print(f"[⚠️ 박스 생성 실패] 절대 X {x_absolute}가 어떤 세그먼트 범위에도 속하지 않아 저장하지 않습니다.")
            # 박스 아이템도 제거해야 함 (마지막에 추가된 아이템이 현재 그리는 아이템) # 이 로직은 BoxDrawingView에서 처리하는 것이 더 안전
            # self.box_view.scene().removeItem(self.box_view.box_items[-1])
            # self.box_view.box_items.pop()
            return # 유효한 세그먼트를 찾지 못하면 저장하지 않음

        # 박스 아이템에 절대 좌표 데이터 저장 (BoxDrawingView에서 생성 시 이미 임시 데이터가 있을 수 있으나, 여기서 최종 확정)
        # 마지막에 추가된 박스 아이템을 찾아 데이터 갱신
        if self.box_view.box_items:
            last_added_item = self.box_view.box_items[-1]
            # rect = last_added_item.rect()
            # if rect.x() == box_data_local["x"] and rect.y() == box_data_local["y"] and rect.width() == box_data_local["width"] and rect.height() == box_data_local["height"]:
            last_added_item.setData(1, box_data_absolute) # data(1)에 절대 좌표 저장
            last_added_item.setData(0, found_segment_id) # data(0)에 세그먼트 ID 저장

        # --- 디버그 로그 추가 ---
        print(f"[박스 생성 디버그]")
        print(f" - 로컬 X (뷰어 기준): {box_data_local['x']}")
        print(f" - 스크롤바 절대 위치: {self.ui.horizontalScrollBar.value()}")
        print(f" - 절대 X: {x_absolute}")
        print(f" → 속한 세그먼트 ID: {found_segment_id}")
        # --- 디버그 로그 끝 ---


        # 파일에 박스 데이터 추가
        # add_box_absolute 함수에 segment_id 인자 추가
        self.box_view.add_box_absolute("GUI_boxes/boxes.json", found_segment_id, box_data_absolute)

    def handle_box_deleted(self, box_data_to_delete):
        """박스 삭제 시 호출되어 파일에서 해당 박스 정보를 삭제"""
        # 삭제될 박스의 절대 좌표 데이터에서 세그먼트 ID 찾기
        x_absolute = box_data_to_delete.get("x_absolute")
        if x_absolute is None:
             print("[⚠️ 박스 삭제 실패] 삭제할 박스 데이터에 절대 좌표가 없습니다.")
             return

        found_segment_id = None
        for idx, config in enumerate(self.inspection_config):
            segment_start = config[4]  # total_length
            segment_length = config[3]  # length_pix
            segment_end = segment_start + segment_length
            # 박스의 시작점 또는 끝점이 세그먼트 범위에 들어오는지 확인
            if (segment_start <= x_absolute < segment_end) or (segment_start <= x_absolute + box_data_to_delete.get("width", 0) <= segment_end):
                 # 편의상 박스의 시작점이 속한 세그먼트로 결정
                 if segment_start <= x_absolute < segment_end:
                      found_segment_id = idx
                      break

        if found_segment_id is None:
             print(f"[⚠️ 박스 삭제 실패] 삭제할 박스의 절대 X {x_absolute}가 어떤 세그먼트 범위에도 속하지 않아 파일에서 찾을 수 없습니다.")
             return # 유효한 세그먼트를 찾지 못하면 파일에서 삭제하지 않음

        # delete_box_absolute 함수에 segment_id 인자 추가
        self.box_view.delete_box_absolute("GUI_boxes/boxes.json", found_segment_id, box_data_to_delete)



    def show_alarm_popup(self):
        dialog = AlarmPopupDialog(self)
        dialog.update_motor_status(120.5, 121.3)  # 필요시 상태 업데이트 ##예시 상태로 일단 표시
        dialog.update_battery_status("85%")
        dialog.update_distance(12.34)
        dialog.exec_()

    def show_setting_popup(self):
        dialog = SettingPopupDialog(self)
        dialog.exec_()

    def show_camera_popup(self):
        dialog = CameraPopupDialog(self)
        dialog.exec_()

    def load_visible_boxes(self, start_x, end_x):
        """현재 보이는 구간의 박스만 로드"""
        try:
            with open("GUI_boxes/boxes.json", 'r') as f:
                data = json.load(f)

            # 기존 박스 제거
            self.box_view.clear_all_boxes()

            # 현재 보이는 구간에 해당하는 세그먼트 찾기
            visible_segments = []
            for idx, config in enumerate(self.inspection_config):
                segment_start = config[4]  # total_length (시작 offset)
                segment_length = config[3]  # length_pix
                segment_end = segment_start + segment_length

                # 세그먼트가 현재 보이는 구간과 겹치는지 확인
                if not (segment_end < start_x or segment_start > end_x):
                    visible_segments.append(idx)

            print(f"[👁️ 보이는 세그먼트] {visible_segments}")

            # 보이는 세그먼트의 박스만 로드
            for segment_id in visible_segments:
                boxes = data.get(str(segment_id), [])
                for box in boxes:
                    # 박스가 현재 보이는 구간에 있는지 확인
                    if start_x <= box["x_absolute"] <= end_x:
                        self.box_view.draw_box_absolute(
                            box["x_absolute"],
                            box["y"],
                            box["width"],
                            box["height"]
                        )

        except Exception as e:
            print(f"[❌ 박스 로드 실패] {e}")
    
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
        elif direction == -1:  # 왼쪽 이동
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
        else:
            pass

    def update_buffer_status(self, buffer_slot, is_valid=True):
        if is_valid:
            self.merged_buffers[buffer_slot][3] = True
            print(f"[✅ 버퍼 {buffer_slot} 활성화]")
        else:
            self.merged_buffers[buffer_slot][3] = False
            self.merged_buffers[buffer_slot][4] = None  # 메모리 정리
            print(f"[🔄 버퍼 {buffer_slot} 비활성화]")

    def build_merged_buffer(self, start_idx, buffer_slot, on_ready_callback=None):
        """버퍼 생성 및 병합"""
        try:
            # ① 실제로 존재하는 인덱스만 추려낸다.
            valid_indices = [
                i for i in range(start_idx, start_idx + 3)
                if 0 <= i < len(self.inspection_config)
            ]
            if not valid_indices:                       # 아예 유효한 이미지가 없으면 종료
                print(f"[⚠️ 버퍼 {buffer_slot}] 유효한 이미지가 없어 병합 생략")
                return
            
            # 버퍼 초기화
            self.merged_buffers[buffer_slot] = [0, 0, 0, False, None]
            
            # 이미지 로딩 순서 보장
            for i, idx in enumerate(valid_indices):
                self.load_requested_image(idx, f"image{i+1}_{buffer_slot}")

            # 이미지 로드 완료 대기
            while not all([
                getattr(self, f"image{i+1}_{buffer_slot}")[1]  # 이미지 로드 완료
                for i in range(len(valid_indices))
            ]):
                time.sleep(0.1)

            imgs = [
                getattr(self, f"image{i+1}_{buffer_slot}")[0]
                for i in range(len(valid_indices))
            ]
            merged = np.concatenate(imgs, axis=1)

            h, w, ch = merged.shape
            qimg = QImage(merged.tobytes(), w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
            qpixmap = QPixmap.fromImage(qimg)

            # 버퍼 정보 업데이트
            start_offset = self.inspection_config[valid_indices[0]][4]
            total_length = sum(self.inspection_config[i][3] for i in valid_indices)
            if valid_indices[-1] == len(self.inspection_config) - 1:  # 마지막 이미지인 경우
                total_length = self.inspection_config[valid_indices[-1]][4] + self.inspection_config[valid_indices[-1]][3] - start_offset
            mid_idx = valid_indices[len(valid_indices) // 2]  # 가운데 이미지

            # 버퍼 정보 설정
            self.merged_buffers[buffer_slot] = [
                start_offset,    # 시작 offset 
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

    def load_boxes_absolute(self, filepath, current_buffer_offset, viewer_width):
        """파일에서 모든 박스를 불러와 현재 버퍼 범위에 해당하는 박스만 그리기"""
        self.clear_all_boxes() # 기존 박스 제거

        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            all_boxes = data.get("boxes", [])

            # 현재 뷰어의 절대 범위 계산
            viewer_abs_start = self.ui.horizontalScrollBar.value()
            viewer_abs_end = viewer_abs_start + viewer_width

            for box_data in all_boxes:
                x_absolute = box_data["x_absolute"]
                y = box_data["y"]
                width = box_data["width"]
                height = box_data["height"]

                # 박스의 절대 범위
                box_abs_start = x_absolute
                box_abs_end = x_absolute + width

                # 박스가 뷰어 범위와 겹치는지 확인
                if not (box_abs_end < viewer_abs_start or box_abs_start > viewer_abs_end):
                    # 현재 버퍼 기준 상대 좌표 계산: 박스 절대 위치 - 현재 버퍼 시작 절대 오프셋
                    x_local = x_absolute - current_buffer_offset

                    # 박스 그리기
                    self.create_box_item(x_local, y, width, height, absolute_data=box_data) # 박스 생성 시 절대 좌표 정보도 함께 저장
                    # print(f"[✨ 박스 그림] 절대 X: {x_absolute}, 상대 X: {x_local}")

        except FileNotFoundError:
            print(f"[ℹ️ 파일 없음] {filepath} 파일 없음")
            pass # 파일이 없으면 박스 없음으로 처리
        except Exception as e:
            print(f"[❌ 박스 로드 실패] {e}")

    def handle_received_image(self, img_data):
        try:
            np_arr = np.frombuffer(img_data, dtype=np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is None:
                print("[⚠️ 이미지 디코드 중...]")
                return

            # 수신된 이미지를 포함하는 모든 버퍼 찾기
            for slot in [0, 1, 2]:
                if not self.merged_buffers[slot][3]:  # 버퍼가 활성화되어 있지 않으면 스킵
                    continue
                
                mid_idx = self.merged_buffers[slot][2]
                if mid_idx is None:
                    continue

                # 현재 버퍼의 이미지 인덱스 범위 확인
                buffer_indices = [mid_idx - 1, mid_idx, mid_idx + 1]
                if self.image_save_index in buffer_indices:
                    # 현재 버퍼에 이미지 반영
                    image_pos = buffer_indices.index(self.image_save_index)
                    attr_name = f"image{image_pos+1}_{slot}"
                    setattr(self, attr_name, [image, True])
                    print(f"[♻️ 실시간 반영] {attr_name} ← 수신 이미지 index={self.image_save_index}")

                    # 현재 보고 있는 버퍼라면 즉시 화면 업데이트
                    if slot == self.current_buffer_index:
                        try:
                            # 현재 버퍼의 이미지들 가져오기
                            imgs = [
                                getattr(self, f"image{i+1}_{slot}")[0]
                                for i in range(3)
                                if getattr(self, f"image{i+1}_{slot}")[0] is not None
                            ]
                            
                            if imgs:
                                # 이미지 병합
                                merged = np.concatenate(imgs, axis=1)
                                h, w, ch = merged.shape
                                bytes_per_line = ch * w
                                qimg = QImage(merged.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                                qpixmap = QPixmap.fromImage(qimg)
                                
                                # 버퍼 정보 업데이트
                                self.merged_buffers[slot][4] = qpixmap
                                
                                # 현재 스크롤 위치 기준으로 화면 업데이트
                                current_scroll = self.ui.horizontalScrollBar.value()
                                start_x = current_scroll - self.merged_buffers[slot][0]
                                if 0 <= start_x <= qpixmap.width() - self.viewer_width:
                                    self.display_pixmap_slice(qpixmap, start_x, self.viewer_width)
                                    print(f"[🖼️ 화면 업데이트] index={self.image_save_index}")
                        except Exception as e:
                            print(f"[⚠️ 실시간 화면 업데이트 실패] {e}")

            # 수신 완료 표시
            self.inspection_config[self.image_save_index][5] = True
            
            # 이미지 저장
            save_path = f"inspection_result/image/{self.image_save_index}.jpg"
            cv2.imwrite(save_path, image)
            print(f"[💾 저장됨] {save_path}")

            self.image_save_index += 1
        
            if self.image_save_index >= len(self.inspection_config):
                print("[✅ 저장 완료]")
                self.image_receiver.stop()

        except Exception as e:
            print(f"[❌ 저장 실패] {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TrackBotApp()

    window.show()
    sys.exit(app.exec_())
