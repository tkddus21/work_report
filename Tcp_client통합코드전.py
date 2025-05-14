import sys
import os
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QGraphicsPixmapItem
from trackbot_gui import Ui_MainWindow
from camera_popup import CameraPopupDialog 
from setting_popup import SettingPopupDialog
from alarm_popup import AlarmPopupDialog
from PyQt5.QtGui import QPixmap, QImage
import numpy as np
import time
import re

import socket
import json
from PyQt5.QtCore import QObject, pyqtSignal, QThread

from collections import OrderedDict

CONTROL_PORT = None
SEND_INAGE_PORT = None

WHITE_IMAGE_PATH = "/tmp/white_placeholder.png"
def generate_white_image(width=4000   , height=4096):
    img = QImage(width, height, QImage.Format_RGB888)
    img.fill(QtGui.QColor("white"))
    img.save(WHITE_IMAGE_PATH)


class TcpImageClientWorker(QThread):
    image_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.save_folder = "/home/krristudent/images"
        os.makedirs(self.save_folder, exist_ok=True)
        self.image_counter = 1

    def recv_exactly(self, size):
        data = b''
        while len(data) < size:
            more = self.sock.recv(size - len(data))
            if not more:
                raise ConnectionError("연결 끊김")
            data += more
        return data

    def run(self):
        while True:  # 무한 루프: 연결 끊기면 다시 시도
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.running = True
                print(f"[TCP-IMG] 서버 연결됨: {self.host}:{self.port}")

                time.sleep(1)

                while self.running:
                    try:
                        size_bytes = self.recv_exactly(4)
                        img_size = int.from_bytes(size_bytes, byteorder='big')
                        print(f"[TCP-IMG] 수신할 이미지 크기: {img_size} bytes")

                        img_data = self.recv_exactly(img_size)

                        save_path = os.path.join(self.save_folder, f"scan_{self.image_counter}.png")
                        with open(save_path, 'wb') as f:
                            f.write(img_data)

                        time.sleep(2)
                        actual_size = os.path.getsize(save_path)
                        print(f"[DEBUG] 저장된 파일 크기: {actual_size}")
                        print(f"[TCP-IMG] 저장 완료: {save_path}")
                        self.image_counter += 1
                        self.image_received.emit(save_path)

                    except Exception as e:
                        self.error_occurred.emit(f"[TCP-IMG RECV 오류] {e}")
                        break  # 내부 루프 탈출 → 연결 종료 → 재연결 시도

            except Exception as e:
                self.error_occurred.emit(f"[TCP-IMG 연결 실패] {e}")

            finally:
                self.running = False
                if self.sock:
                    try:
                        self.sock.shutdown(socket.SHUT_RDWR)
                    except:
                        pass
                    self.sock.close()
                    self.sock = None
                print("[TCP-IMG] 연결 종료됨. 3초 후 재연결 시도...")
                time.sleep(3)



    def close(self):
        self.running = False
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self.sock.close()
            self.sock = None

class TcpClientWorker(QThread):
    # 👉 워커 내부 신호
    distance_received = pyqtSignal(float)
    rpm_received = pyqtSignal(float, float)
    image_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.waiting_for = "header"
        self.header = None

    def run(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.running = True
                print(f"[TCP] 서버 연결됨: {self.host}:{self.port}")

                buffer = ""
                while self.running:
                    try:
                        raw_data = self.sock.recv(1024)
                        if not raw_data:
                            raise ConnectionError("서버 연결 종료됨")

                        try:
                            data = raw_data.decode()
                        except UnicodeDecodeError as e:
                            self.error_occurred.emit(f"[RECV 디코딩 오류]: {e}")
                            break

                        buffer += data
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.handle_line(line.strip())

                    except Exception as e:
                        self.error_occurred.emit(f"[RECV 예외] {e}")
                        break

            except Exception as e:
                self.error_occurred.emit(f"[연결 실패] {e}")

            finally:
                self.running = False
                if self.sock:
                    try:
                        self.sock.shutdown(socket.SHUT_RDWR)
                    except:
                        pass
                    self.sock.close()
                    self.sock = None

                print(f"[TCP] {self.port} 재연결 대기 중...")
                time.sleep(3)  # 재시도 대기


    def handle_line(self, line):
        if self.waiting_for == "header":
            try:
                self.header = json.loads(line)
                self.waiting_for = "body"
            except Exception as e:
                self.error_occurred.emit(f"[헤더 파싱 오류] {e}")

        elif self.waiting_for == "body":
            try:
                body = json.loads(line)
                checksum = body.pop("checksum", -1)
                body_str = json.dumps(body)
                expected = self.calculate_checksum(body_str)

                if checksum != expected:
                    print("⚠️ checksum mismatch")
                    self.waiting_for = "header"
                    return

                self.process_message(self.header["type"], body)
                self.waiting_for = "header"
            except Exception as e:
                self.error_occurred.emit(f"[본문 파싱 오류] {e}")
                self.waiting_for = "header"

    def process_message(self, type_str, body):
        if type_str == "distance":
            self.distance_received.emit(body["value"])
        elif type_str == "wheel_rpm":
            self.rpm_received.emit(body["left"], body["right"])
        elif type_str == "image":
            self.image_received.emit(body["data"])
        else:
            self.error_occurred.emit(f"알 수 없는 타입: {type_str}")

    def calculate_checksum(self, data_str):
        return sum(bytearray(data_str.encode())) % 256

    def send_rpm(self, left: float, right: float):
        if self.sock is None:
            self.error_occurred.emit("소켓이 열려있지 않음")
            return

        body_dict = {"left": left, "right": right}
        body_json = json.dumps(body_dict)
        header = {"type": "cmd_rpm", "length": len(body_json.encode())}

        checksum = self.calculate_checksum(body_json)
        body_dict["checksum"] = checksum
        full_body_json = json.dumps(body_dict)

        try:
            self.sock.sendall((json.dumps(header) + '\n').encode())
            self.sock.sendall((full_body_json + '\n').encode())
            print(f"[SEND] L={left}, R={right}")
        except Exception as e:
            self.error_occurred.emit(f"[송신 오류] {e}")

    def close(self):
        self.running = False
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self.sock.close()
            self.sock = None


class TcpClient(QObject):
    # 👉 외부에 노출하는 신호
    distance_received = pyqtSignal(float)
    rpm_received = pyqtSignal(float, float)
    image_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: int, host: str = "0.tcp.jp.ngrok.io"):
        super().__init__()
        self.worker = TcpClientWorker(host, port)

        # 신호 연결
        self.worker.distance_received.connect(self.distance_received)
        self.worker.rpm_received.connect(self.rpm_received)
        self.worker.image_received.connect(self.image_received)
        self.worker.error_occurred.connect(self.error_occurred)

    def connect_to_server(self):
        self.worker.start()

    def send_rpm(self, left: float, right: float):
        self.worker.send_rpm(left, right)

    def disconnect(self):
        self.worker.close()
        self.worker.wait()  # 스레드 완전히 종료 대기


#Pixmap optimization
class PixmapCache:
    def __init__(self, max_size=10):
        self.cache = OrderedDict()
        self.max_size = max_size

    def get_pixmap(self, path):
        if path in self.cache:
            pixmap = self.cache.pop(path)
            self.cache[path] = pixmap
            return pixmap

        pixmap = QPixmap(path)
        if pixmap.isNull():
            return None

        if len(self.cache) >= self.max_size:
            self.cache.popitem(last=False)  # 가장 오래된 것 제거

        self.cache[path] = pixmap
        return pixmap
    
    def invalidate(self, path):

        if path in self.cache:
            del self.cache[path]


# 📌 커스텀 QGraphicsView: 마우스 위치 따라 확대 이미지 표시
class LineCameraView(QtWidgets.QGraphicsView):
    
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)

        # 확대용 아이템 (기존 기능 유지)
        self.zoom_item = None

        # 박스 그리기 상태
        self.drawing_box = False
        self.start_pos = None
        self.rubber_band_item = None

        # 박스 관리
        self.box_items = []
        self.selected_box = None



    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())

            # 기존 박스 클릭했는지 확인
            clicked_items = self.scene().items(scene_pos)
            for item in clicked_items:
                if isinstance(item, QtWidgets.QGraphicsRectItem):
                    self.select_box(item)
                    return

            self.clear_selection()

            # 새 박스 그리기 시작
            self.drawing_box = True
            self.start_pos = scene_pos

            # 이전 박스 제거
            if self.rubber_band_item:
                self.scene().removeItem(self.rubber_band_item)
                self.rubber_band_item = None



    def mouseMoveEvent(self, event):
    # 🔍 기본 확대 기능 유지
        super().mouseMoveEvent(event)
        
        # blue 박스 드로잉 (기존 그대로 유지)
        if self.drawing_box and self.start_pos:
            end_pos = self.mapToScene(event.pos())
            rect = QtCore.QRectF(self.start_pos, end_pos).normalized()

            if self.rubber_band_item:
                self.rubber_band_item.setRect(rect)
            else:
                pen = QtGui.QPen(QtGui.QColor(0, 120, 255), 2, QtCore.Qt.DashLine)
                self.rubber_band_item = self.scene().addRect(rect, pen)

        if not self.zoom_item:
            return

        scene_pos = self.mapToScene(event.pos())
        items = self.scene().items(scene_pos)

        for item in items:
            if isinstance(item, QGraphicsPixmapItem):
                pixmap = item.pixmap()
                local_pos = item.mapFromScene(scene_pos)

                x = int(local_pos.x())
                y = int(local_pos.y())

                # 🔍 확대 박스 크기 (정사각형)
                zoom_box_size = 100
                half_size = zoom_box_size // 2

                # 🔍 확대 영역 사각형 (이미지 범위 넘지 않도록 intersect)
                rect = QtCore.QRect(x - half_size, y - half_size, zoom_box_size, zoom_box_size)
                rect = rect.intersected(pixmap.rect())

                if rect.isValid():
                    cropped = pixmap.copy(rect)

                    # 확대 이미지 보여줄 뷰포트 크기 가져오기
                    view_size = self.zoom_item.scene().views()[0].viewport().size()
                    target_width = view_size.width()
                    target_height = view_size.height()

                    # ✅ 확대 이미지 비율 유지하며 꽉 채우기
                    scaled = cropped.scaled(
                        target_width, target_height,
                        QtCore.Qt.KeepAspectRatioByExpanding,
                        QtCore.Qt.SmoothTransformation
                    )

                    # ✅ 확대 이미지 중앙 정렬
                    offset_x = (target_width - scaled.width()) / 2
                    offset_y = (target_height - scaled.height()) / 2

                    self.zoom_item.setPixmap(scaled)
                    self.zoom_item.setPos(offset_x, offset_y)

                    # 확대 이미지 신호 발생 (필요 시 연결 가능)
                    if hasattr(self.parent(), "zoom_updated"):
                        self.parent().zoom_updated.emit(scaled)
                return

        # 확대할 이미지가 없으면 클리어
        self.zoom_item.setPixmap(QtGui.QPixmap())


    def mouseReleaseEvent(self, event):
        if self.drawing_box and event.button() == QtCore.Qt.LeftButton:
            self.drawing_box = False
            end_pos = self.mapToScene(event.pos())
            rect = QtCore.QRectF(self.start_pos, end_pos).normalized()

            # 너무 작은 건 무시
            if rect.width() > 5 and rect.height() > 5:
                box = self.scene().addRect(rect, QtGui.QPen(QtGui.QColor(255, 0, 0), 2))
                box.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable)
                box.setZValue(1)
                self.box_items.append(box)

                # ✅ 자동 저장
                if hasattr(self.window(), "save_boxes_for_image") and hasattr(self.parent(), "current_img_index"):
                    self.window().save_boxes_for_image()
            # 임시 박스 제거
            if self.rubber_band_item:
                self.scene().removeItem(self.rubber_band_item)
                self.rubber_band_item = None

            self.start_pos = None

        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete and self.selected_box:
            self.scene().removeItem(self.selected_box)
            if self.selected_box in self.box_items:
                self.box_items.remove(self.selected_box)
            self.selected_box = None

            # ✅ 자동 저장
            if hasattr(self.window(), "save_boxes_for_image") and hasattr(self.parent(), "current_img_index"):
                self.window().save_boxes_for_image()
        else:
            super().keyPressEvent(event)

    def select_box(self, box_item):
        if self.selected_box:
            self.selected_box.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0), 2))  # 선택 해제 시 빨간색 복귀

        self.selected_box = box_item
        box_item.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0), 2))  # 선택 시 초록색

    def clear_selection(self):
        if self.selected_box:
            self.selected_box.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0), 2))
            self.selected_box = None



# 🔧 배경 이미지 생성 유틸  #width , height value 변경가능하게
# green, red image 채우는 함수
def generate_green_background(value: float, width=3200, height=120) -> QtGui.QPixmap:
    image = QtGui.QImage(width, height, QtGui.QImage.Format_ARGB32)
    image.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(image)
    painter.setBrush(QtGui.QColor(0, 255, 0, 120))  # 반투명 녹색
    painter.setPen(QtCore.Qt.NoPen)
    painter.drawRect(0, 0, int(width * value), height)
    painter.end()
    return QtGui.QPixmap.fromImage(image)


def generate_red_lines_image(values: list[float], width=3200, height=120) -> QtGui.QPixmap:
    image = QtGui.QImage(width, height, QtGui.QImage.Format_ARGB32)
    image.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(image)
    pen = QtGui.QPen(QtGui.QColor(255, 0, 0, 180))
    pen.setWidth(4)
    painter.setPen(pen)

    for v in values:
        if 0.0 <= v <= 1.0:  # ✅ 안전 체크
            x = int(v * width)
            painter.drawLine(x, 0, x, height)
    painter.end()
    return QtGui.QPixmap.fromImage(image)



class TrackBotApp(QMainWindow):
    zoom_updated = QtCore.pyqtSignal(QtGui.QPixmap)
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("TrackBot GUI")

        self.image_num = 10  # 기본 이미지 수

        # 미리 white placeholder 이미지 준비
        generate_white_image()

        # 이미지 디렉토리
        self.image_dir = "/home/krristudent/images"  # 원하시는 경로로 바꿔도 됩니다
        os.makedirs(self.image_dir, exist_ok=True)

        self.image_paths = self.load_images_from_directory(self.image_dir)
        while len(self.image_paths) < self.image_num:
            placeholder = os.path.join(self.image_dir, f"scan_{len(self.image_paths)+1}.png")
            QtGui.QImage(WHITE_IMAGE_PATH).save(placeholder)
            self.image_paths.append(placeholder)


        self.update_tool_button_style()  # 초기 스타일 강제 적용

        self.pixmap_cache = PixmapCache(max_size=10)

        # 창 크기 설정
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(1400, 800)

        central_widget = self.centralWidget()
        if central_widget:
            central_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.popup = None
        self.setting_popup = None
        self.alarm_popup = None


        self.box_data = {}  # 이미지 인덱스 ➜ 박스 리스트 저장


        # 녹화 버튼 토글
        self.ui.toolButton.setCheckable(True)
        self.ui.toolButton.clicked.connect(self.toggle_recording)

        # rpm_tcp
        self.rpm_timer = QtCore.QTimer()
        self.rpm_timer.timeout.connect(self.send_rpm_tcp)
        self.rpm_timer.start(100)  # 100ms(0.1sce) 간격


        # 🎮 조이스틱 슬라이더 초기화 및 복귀
        self.ui.left.setMinimum(-100)
        self.ui.left.setMaximum(100)
        self.ui.left.setValue(0)
        self.ui.left.sliderReleased.connect(lambda: self.ui.left.setValue(0))
        self.ui.left.valueChanged.connect(self.send_rpm_tcp)

        self.ui.right.setMinimum(-100)
        self.ui.right.setMaximum(100)
        self.ui.right.setValue(0)
        self.ui.right.sliderReleased.connect(lambda: self.ui.right.setValue(0))
        self.ui.right.valueChanged.connect(lambda v: print(f"[🔍 줌] 값: {v}"))


        # 방향 버튼
        self.ui.up_button.pressed.connect(lambda: self.send_command("move up"))
        self.ui.up_button.released.connect(lambda: self.send_command("stop"))
        self.ui.down_button.pressed.connect(lambda: self.send_command("move down"))
        self.ui.down_button.released.connect(lambda: self.send_command("stop"))
        self.ui.left_button.pressed.connect(lambda: self.send_command("move left"))
        self.ui.left_button.released.connect(lambda: self.send_command("stop"))
        self.ui.right_button.pressed.connect(lambda: self.send_command("move right"))
        self.ui.right_button.released.connect(lambda: self.send_command("stop"))



        # 명령 버튼
        self.ui.auto_drive_button.clicked.connect(lambda: self.send_command("🚗 자율 주행 시작"))
        self.ui.alarm_button.clicked.connect(lambda: self.send_command("🔔 알람 발동"))
        self.ui.return_button.clicked.connect(lambda: self.send_command("↩️ 복귀 명령"))
        self.ui.setting_button.clicked.connect(lambda: self.send_command("⚙️ 설정 실행"))



        # 팝업 열기
        self.ui.alarm_button.clicked.connect(self.toggle_alarm_popup)
        self.ui.setting_button.clicked.connect(self.toggle_setting_popup)
        

        #도면 번호 전용 Scene 추가
        self.image_scene = QtWidgets.QGraphicsScene()
        self.label_scene = QtWidgets.QGraphicsScene()
        self.custom_line_camera = LineCameraView(self)

        self.custom_line_camera.setScene(self.image_scene)
        self.custom_line_camera.setSizePolicy(QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        # QSplitter 기반 교체
        splitter: QtWidgets.QSplitter = self.ui.leftSplitter
        index = splitter.indexOf(self.ui.LineCamera)
        splitter.insertWidget(index, self.custom_line_camera)
        self.ui.LineCamera.deleteLater()
        self.ui.LineCamera = self.custom_line_camera

        
        # 📍 스크롤 제거 설정 추가!
        self.custom_line_camera.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.custom_line_camera.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        
        
        # 🔎 EventCamera 확대 이미지용 Scene 구성
        self.zoom_scene = QtWidgets.QGraphicsScene()
        self.zoom_item = QtWidgets.QGraphicsPixmapItem()
        self.zoom_scene.addItem(self.zoom_item)
        self.ui.EventCamera.setScene(self.zoom_scene)
        self.custom_line_camera.zoom_item = self.zoom_item

        
        # 🎯 확대 뷰 스크롤 제거
        self.ui.EventCamera.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ui.EventCamera.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        
        # 도면 번호 뷰 설정
        self.ui.drawingNumberView.setScene(self.label_scene)


        # 🎯 도면 뷰 스크롤 제거
        self.ui.drawingNumberView.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ui.drawingNumberView.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)


        self.image_paths = self.load_images_from_directory(self.image_dir)
 

        #problem
        sample_pixmap = QPixmap(self.image_paths[0])
        self.image_width = sample_pixmap.width()
        self.image_height = sample_pixmap.height()

        # 재설정
        self.image_scene = QtWidgets.QGraphicsScene()
        self.label_scene = QtWidgets.QGraphicsScene()
        self.ui.LineCamera.setScene(self.image_scene)
        self.ui.drawingNumberView.setScene(self.label_scene)



        # 초기 렌더링 완료 후 슬라이딩 갱신
        QtCore.QTimer.singleShot(0, lambda: self.update_sliding_window(self.ui.horizontalScrollBar.value()))
        self.ui.horizontalScrollBar.setStyleSheet("background: transparent;")
        self.ui.horizontalScrollBar_2.setStyleSheet("background: transparent;")

        # 슬라이더 위에 이미지용 QLabel 추가
        self.green_background_label = QtWidgets.QLabel(self.ui.sliderContainer)
        self.redline_background_label = QtWidgets.QLabel(self.ui.sliderContainer)

        self.green_background_label.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.redline_background_label.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)

        # QLabel이 뒤로 가게
        self.green_background_label.lower()
        self.redline_background_label.lower()

        # 배경 투명 처리
        self.ui.horizontalScrollBar.setStyleSheet("background: transparent;")
        self.ui.horizontalScrollBar_2.setStyleSheet("background: transparent;")

        # ✅ 여기서 호출! QLabel이 이미 정의된 상태
        self.setup_scene_with_sliding_window()
        

        # 초기 슬라이딩 및 이미지
        QtCore.QTimer.singleShot(0, lambda: self.update_sliding_window(self.ui.horizontalScrollBar.value()))
        self.green_background_label.setPixmap(generate_green_background(0.0))
        self.redline_background_label.setPixmap(generate_red_lines_image([]))

        #슬라이더 반투명 그린 채우는 함수(비율 0~1로 그림)
        self.update_green_slide(0.9)

        #슬라이더 레드 라인 그리는 함수(비율 0~1로  그림)
        self.update_red_lines([0.15, 0.9])

        #TCP 소켓 연결 추가
        self.max_rpm = 300.0  # 최대 RPM 값
        self.tcp_connected = False

        # # TCP 연결 설정

        self.tcp_client = TcpClient(port=CONTROL_PORT)
        self.tcp_client.connect_to_server()
        
        self.tcp_client.distance_received.connect(self.handle_distance)
        self.tcp_client.rpm_received.connect(self.handle_rpm)
        self.tcp_client.error_occurred.connect(lambda msg: print(f"[TCP 오류] {msg}"))


        # TrackBotApp.__init__ 내부에 추가
        self.image_client = TcpImageClientWorker(host="0.tcp.jp.ngrok.io", port=14654)
        self.image_client.image_received.connect(self.handle_image_received)
        self.image_client.error_occurred.connect(lambda msg: print(f"[TCP-IMG 오류] {msg}"))
        self.image_client.start()

    
    
    def handle_image_received(self, path: str):
        print(f"[GUI] 이미지 수신 완료: {path}")
         
        # 캐시 무효화
        self.pixmap_cache.invalidate(path)

        # 현재 슬라이더 값으로 이미지 다시 렌더링
        self.update_sliding_window(self.ui.horizontalScrollBar.value())


    def handle_distance(self, value: float):
        print(f"[수신] 총 주행 거리: {value:.2f} m")
        if self.alarm_popup:
            self.alarm_popup.update_distance(value)

    def handle_rpm(self, left: float, right: float):
        print(f"[수신] 좌우 RPM: L={left:.1f}, R={right:.1f}")
        if self.alarm_popup:
            self.alarm_popup.update_motor_status(left, right)
            

    # 거리 정보: {"type": "distance", "value": float}
    # 바퀴 RPM: {"type": "wheel_rpm", "left": float, "right": float}
    def handle_tcp_data(self, data_str):
        try:
            json_obj = json.loads(data_str)
            msg_type = json_obj.get("type")

            if msg_type == "distance":
                distance = json_obj.get("value", None)
                if distance is not None:
                    print(f"[수신] 총 주행 거리: {distance:.2f} m")
                    if self.alarm_popup:
                        self.alarm_popup.update_distance(distance)
                    # 예: 거리 표시용 UI에 업데이트 가능
                    # self.ui.distance_label.setText(f"{distance:.2f} m")

            elif msg_type == "wheel_rpm":
                left = json_obj.get("left", None)
                right = json_obj.get("right", None)
                if left is not None and right is not None:
                    print(f"[수신] 좌우 RPM: L={left:.1f}, R={right:.1f}")
                    if self.alarm_popup:
                        self.alarm_popup.update_motor_status(left, right)
                    # 예: 상태 표시, 그래프 반영 등 가능
                    # self.ui.rpm_label.setText(f"L: {left:.1f}, R: {right:.1f}")

            else:
                print(f"[경고] 알 수 없는 타입: {msg_type}")

        except json.JSONDecodeError:
            print("[에러] JSON 파싱 실패")
        except Exception as e:
            print(f"[예외] 처리 중 오류 발생: {e}")


    # 슬라이딩 윈도우 🚗 RPM 전송
    def send_rpm_tcp(self):
        rpm = int((self.ui.left.value() / 100.0) * self.max_rpm)       
        self.tcp_client.send_rpm(left=rpm, right=rpm)
        print(f"🚗 RPM 전송: {rpm:}")


    # 슬라이더 녹색 비율 업데이트
    def update_green_slide(self, input : float):
        green_ratio = input
        self.green_background_label.setPixmap(
            generate_green_background(
                green_ratio,
                width=self.ui.horizontalScrollBar.width(),
                height=self.ui.horizontalScrollBar.height()
            )
        )
    
    

    def update_red_lines(self, float_list: list[float]):
        red_image = generate_red_lines_image(
            values=float_list,
            width=self.ui.horizontalScrollBar_2.width(),
            height=self.ui.horizontalScrollBar_2.height()
        )
        self.redline_background_label.setPixmap(red_image)
                


    # 추후 실시간으로 분석이 완료되거나, 이미지가 취합될 때 갱신
    def set_collected_images(self, count: int):
        self.collected_images = count
        self.update_green_slide()

    #박스 데이터를 로딩
    def load_boxes_for_image(self, img_index: int):
        return self.box_data.get(img_index, [])


    # 이미지 디렉토리에서 그리기
    #offset은 슬라이더 위치에 따라 조정
    def draw_boxes_on_scene(self, boxes, scene_width, scene_height, x_offset=0, img_index=None):
        
        for idx, (xc, yc, w, h) in enumerate(boxes):
            # 좌표 계산
            box_w = w * scene_width
            box_h = h * scene_height
            top_left_x = (xc - w / 2) * scene_width + x_offset
            top_left_y = (yc - h / 2) * scene_height

            rect = QtCore.QRectF(top_left_x, top_left_y, box_w, box_h)
            pen = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
            box = self.image_scene.addRect(rect, pen)
            box.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable)
            box.setZValue(2)
            self.custom_line_camera.box_items.append(box)

            # 🔢 박스 위에 텍스트 추가
            if img_index is not None:
                text_item = QtWidgets.QGraphicsTextItem(f"{img_index + 1}")
                font = QtGui.QFont("Arial", 10, QtGui.QFont.Bold)
                text_item.setFont(font)
                text_item.setDefaultTextColor(QtGui.QColor("blue"))
                text_item.setPos(top_left_x + 5, top_left_y + 5)
                text_item.setZValue(3)
                self.image_scene.addItem(text_item)
            
        
        




    # 박스 저장
    def save_boxes_for_image(self):
        temp_box_data = {}
        
        for item in self.custom_line_camera.box_items:
            if not isinstance(item, QtWidgets.QGraphicsRectItem):
                continue

            rect = item.rect()
            top_left = item.scenePos()
            center_x = top_left.x() + rect.x() + rect.width() / 2
            center_y = top_left.y() + rect.y() + rect.height() / 2

            # 🧠 정확한 이미지 인덱스 찾기
            matched_img_index = None
            for img_index, offset in enumerate(self.image_offsets):
                if img_index >= len(self.scaled_pixmaps):
                    continue
                img_width = self.scaled_pixmaps[img_index].width()
                if offset <= center_x < offset + img_width:
                    matched_img_index = img_index
                    break

            if matched_img_index is None:
                print(f"⚠️ 박스 위치가 이미지 범위를 벗어남: center_x={center_x}")
                continue

            pixmap = self.scaled_pixmaps[matched_img_index]
            img_offset = self.image_offsets[matched_img_index]
            img_width = pixmap.width()
            img_height = pixmap.height()

            # 정규화된 좌표계로 저장 (YOLO 형식 유사)
            local_x = center_x - img_offset
            x_center = local_x / img_width
            y_center = center_y / img_height
            width = rect.width() / img_width
            height = rect.height() / img_height

            temp_box_data.setdefault(matched_img_index, []).append((x_center, y_center, width, height))

        # ✅ 병합 저장
        for img_index, boxes in temp_box_data.items():
            self.box_data[img_index] = boxes

        # ✅ 디버그 출력
        print("\n📦 박스 저장 완료:")
        if not self.box_data:
            print("  (박스 없음)")
        for idx in sorted(self.box_data):
            print(f"  - 이미지 {idx + 1} ({self.image_paths[idx]}):")
            for i, (x, y, w, h) in enumerate(self.box_data[idx]):
                print(f"    [{i}] x={x:.3f}, y={y:.3f}, w={w:.3f}, h={h:.3f}")



        # 내보내기 함수 .txt로 저장하고 싶을 때를 위해 export용 함수 (추후에)
        def export_boxes_to_txt(self, output_dir="boxes_export"):
            os.makedirs(output_dir, exist_ok=True)
            for idx, boxes in self.box_data.items():
                with open(os.path.join(output_dir, f"{idx + 1}.txt"), "w") as f:
                    for x, y, w, h in boxes:
                        f.write(f"{x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")
            print(f"📁 박스 내보내기 완료! 폴더: {output_dir}")


    # 이미지 디렉토리에서 이미지 로드
    def load_images_from_directory(self, directory):
        supported_ext = (".png", ".jpg", ".jpeg", ".bmp", ".gif")

        def extract_number(filename):
            match = re.search(r'\d+', filename)
            return int(match.group()) if match else float('inf')

        files = [
            os.path.join(directory, f)
            for f in os.listdir(directory)
            if f.lower().endswith(supported_ext)
        ]
        
        return sorted(files, key=lambda x: extract_number(os.path.basename(x)))


    # 슬라이더 위치에 따라 이미지 씬 업데이트
    def setup_scene_with_sliding_window(self):
        self.image_offsets = [] #각 이미지의 시작 X좌표
        self.scaled_pixmaps = []
        self.total_width = 0
        self.image_scene.clear()

        view_height = self.ui.LineCamera.height()
        offset = 0

        for path in self.image_paths:
            pixmap = self.pixmap_cache.get_pixmap(path)
            if not pixmap or pixmap.isNull():
                continue

            scaled = pixmap.scaledToHeight(view_height, QtCore.Qt.SmoothTransformation)
            self.scaled_pixmaps.append(scaled)

            self.image_offsets.append(offset)
            offset += scaled.width()

        self.total_width = offset
        self.image_scene.setSceneRect(0, 0, self.total_width, view_height)

        self.ui.horizontalScrollBar.setMinimum(0)
        self.ui.horizontalScrollBar.setMaximum(1000)
        self.ui.horizontalScrollBar.valueChanged.connect(self.update_sliding_window)


  
        self.ui.horizontalScrollBar_2.setMinimum(0)
        self.ui.horizontalScrollBar_2.setMaximum(1000)
        self.ui.horizontalScrollBar_2.valueChanged.connect(
            lambda v: print(f"서브 슬라이더 위치: {v}")
        )
  




    def update_sliding_window(self, value):
        # ✅ 슬라이더 이동 전에 박스 저장
        if hasattr(self, "save_boxes_for_image"):
            self.save_boxes_for_image()

        if not self.image_paths:
            return

        self.image_scene.clear()
        self.label_scene.clear()
        self.custom_line_camera.box_items.clear()

        # 전체 필름 너비 기준 스크롤 위치 계산
        scroll_ratio = value / 1000.0
        scroll_x = scroll_ratio * self.total_width

        # self.ui.LineCamera.centerOn(scroll_x, self.ui.LineCamera.height() / 2)  
        # #or
        middle_y = max((scaled.height() for scaled in self.scaled_pixmaps), default=0) / 2
        self.ui.LineCamera.centerOn(scroll_x, middle_y)
        self.ui.drawingNumberView.centerOn(scroll_x, 0) #라벨 뷰 동기화

        font_size = max(10, int(14 * (self.width() / 1920)))
        font = QtGui.QFont("Arial", font_size, QtGui.QFont.Bold)

        # 현재 기준 위치에서 근처 이미지들만 박스/라벨 표시
        visible_width = self.ui.LineCamera.width()
        left_bound = scroll_x - visible_width
        right_bound = scroll_x + visible_width

        for i, path in enumerate(self.image_paths):
            if i >= len(self.image_offsets):
                continue

            offset = self.image_offsets[i]
            if offset > right_bound or (offset + self.image_width) < left_bound:
                continue  # 화면 밖 이미지 제외

            pixmap = self.pixmap_cache.get_pixmap(path)
            if pixmap.isNull():
                continue

            scaled = self.scaled_pixmaps[i] # 미리 스케일된 pixmap 사용
            item = QGraphicsPixmapItem(scaled)
            item.setPos(offset, (self.ui.LineCamera.height() - scaled.height()) / 2)
            self.image_scene.addItem(item)

            #  박스 그리기
            boxes = self.load_boxes_for_image(i)
            self.draw_boxes_on_scene(boxes, scaled.width(), scaled.height(), offset, img_index=i)

            #  라벨 위치: 이미지 중심 기준
            center_x = offset + scaled.width() / 2
            label = QtWidgets.QGraphicsTextItem(f"<== 도면 {i + 1} ==>")
            label.setFont(font)
            label.setDefaultTextColor(QtGui.QColor("black"))
            label_y = (self.ui.drawingNumberView.height() - label.boundingRect().height()) / 2
            label.setPos(center_x - label.boundingRect().width() / 2, label_y)
            self.label_scene.addItem(label)

            label_back = QtWidgets.QGraphicsTextItem(f"{i + 1}")
            label_back.setFont(font)
            label_back.setDefaultTextColor(QtGui.QColor("black"))
            right_x = offset + scaled.width()
            label_back.setPos(right_x - label_back.boundingRect().width(), label_y)
            self.label_scene.addItem(label_back)


        # 전체 씬 크기 재설정 (고정값 아님!)
        self.image_scene.setSceneRect(0, 0, self.total_width, self.ui.LineCamera.height())
        self.label_scene.setSceneRect(0, 0, self.total_width, self.ui.drawingNumberView.height())

    
    # window 크기 조정 시
    def resizeEvent(self, event):
        super().resizeEvent(event)
        
        self.setup_scene_with_sliding_window()
        
        # 위치 재배치
        self.green_background_label.setGeometry(self.ui.horizontalScrollBar.geometry())
        self.redline_background_label.setGeometry(self.ui.horizontalScrollBar_2.geometry())
        
        #  녹색 슬라이드 재갱신
        self.update_green_slide(0.9)
        
     
        QtCore.QTimer.singleShot(0, self.update_tool_button_style)
        QtCore.QTimer.singleShot(0, self.update_fonts_responsive)
        QtCore.QTimer.singleShot(0, self.update_button_min_sizes)
        QtCore.QTimer.singleShot(0, self.update_slider_label_fonts)
        QtCore.QTimer.singleShot(0, self.update_slider_sizes)
        QtCore.QTimer.singleShot(0, self.update_label_view_height)
        QtCore.QTimer.singleShot(0, lambda: self.update_sliding_window(self.ui.horizontalScrollBar.value()))
        
        # resizeEvent 끝에 안전하게 호출
        #  red 슬라이드 재갱신
        QtCore.QTimer.singleShot(0, lambda: self.update_red_lines([0.15, 0.9]))

    def update_label_view_height(self):
        total_height = self.height()
        label_height = max(30, int(total_height * 0.07))
        self.ui.drawingNumberView.setMinimumHeight(label_height)
        self.ui.drawingNumberView.setMaximumHeight(label_height)



    def send_command(self, cmd):
        print(f"[🛰️ 명령] {cmd}")

    def toggle_recording(self):
        if self.ui.toolButton.isChecked():
            self.send_command("촬영 시작")
            if self.popup is None:
                self.popup = CameraPopupDialog(self)
                self.popup.setAttribute(QtCore.Qt.WA_DeleteOnClose)
                self.popup.destroyed.connect(self.popup_closed)
                self.popup.show()
        else:
            self.send_command("촬영 취소")
            if self.popup:
                self.popup.close()
                self.popup = None

    def popup_closed(self):
        self.ui.toolButton.setChecked(False)
        self.update_tool_button_style()
        self.popup = None

    def toggle_setting_popup(self):
        if self.setting_popup is None:
            self.setting_popup = SettingPopupDialog(self)
            self.setting_popup.setAttribute(QtCore.Qt.WA_DeleteOnClose)
            self.setting_popup.destroyed.connect(lambda: self.clear_popup("setting"))
            self.setting_popup.show()
        else:
            self.setting_popup.close()
            self.setting_popup = None

    def toggle_alarm_popup(self):
        if self.alarm_popup is None:
            self.alarm_popup = AlarmPopupDialog(self)
            self.alarm_popup.setAttribute(QtCore.Qt.WA_DeleteOnClose)
            self.alarm_popup.destroyed.connect(lambda: self.clear_popup("alarm"))
            self.alarm_popup.show()
        else:
            self.alarm_popup.close()
            self.alarm_popup = None

    def clear_popup(self, name):
        if name == "setting":
            self.setting_popup = None
        elif name == "alarm":
            self.alarm_popup = None

    def update_tool_button_style(self):
        w = self.ui.toolButton.width()
        h = self.ui.toolButton.height()
        r = min(w, h) // 2
        self.ui.toolButton.setStyleSheet(f"""
            QToolButton {{
                border-radius: {r}px;
                border: 2px solid red;
                background-color: white;
            }}
            QToolButton:checked {{
                background-color: red;
                border: 2px solid black;
                border-radius: {r}px;
            }}
        """)

    def update_slider_label_fonts(self):
        width_scale = self.width() / 1260
        font_size = max(10, int(14 * width_scale))
        font = QtGui.QFont()
        font.setPointSize(font_size)
        for label in [
            self.ui.leftLabelTop, self.ui.leftLabelBottom,
            self.ui.rightLabelTop, self.ui.rightLabelBottom
        ]:
            label.setFont(font)

    def update_slider_sizes(self):
        base_height = 600
        base_width = 1260
        height_scale = self.height() / base_height
        width_scale = self.width() / base_width
        new_height = int(250 * height_scale)
        new_width = int(14 * width_scale)
        for slider in [self.ui.left, self.ui.right]:
            slider.setMinimumHeight(new_height)
            slider.setStyleSheet(f"""
                QSlider::groove:vertical {{
                    background: #ccc;
                    width: {new_width}px;
                }}
                QSlider::handle:vertical {{
                    background: #116688;
                    border-radius: 20px;
                    height: 30px;
                }}
            """)

    def update_button_min_sizes(self):
        base_width = 1260
        scale = self.width() / base_width
        new_min_size = int(60 * scale)
        for btn in [
            self.ui.auto_drive_button, self.ui.alarm_button, self.ui.return_button,
            self.ui.setting_button, self.ui.up_button, self.ui.down_button,
            self.ui.left_button, self.ui.right_button, self.ui.toolButton
        ]:
            btn.setMinimumSize(new_min_size, new_min_size)

    def update_fonts_responsive(self):
        base_width = 1260
        current_width = self.width()
        scale_factor = current_width / base_width
        base_font_size = 14
        scaled_font_size = int(base_font_size * scale_factor)
        sample_button = self.ui.auto_drive_button
        max_font_size_by_height = int(sample_button.height() * 0.4)
        final_font_size = max(10, min(scaled_font_size, max_font_size_by_height))
        widgets = [
            self.ui.auto_drive_button,
            self.ui.alarm_button,
            self.ui.return_button,
            self.ui.setting_button,
            self.ui.up_button,
            self.ui.down_button,
            self.ui.left_button,
            self.ui.right_button,
            self.ui.toolButton,
        ]
        for widget in widgets:
            font = widget.font()
            font.setPointSize(final_font_size)
            widget.setFont(font)

def prompt_for_port(name="포트"):
    while True:
        port_str = input(f"{name} 번호를 입력하세요 (또는 'q' 입력 시 종료): ").strip()
        if port_str.lower() == 'q':
            print("❌ 사용자 종료 요청")
            sys.exit(0)
        if port_str.isdigit():
            return int(port_str)
        print("❌ 잘못된 입력입니다. 숫자를 입력하세요.")

if __name__ == "__main__":
    CONTROL_PORT = prompt_for_port("CONTROL 포트")
    SEND_IMAGE_PORT = prompt_for_port("이미지 전송 포트")

    app = QApplication(sys.argv)
    window = TrackBotApp()

    # 포트 적용
    window.tcp_client = TcpClient(port=CONTROL_PORT, host="ap.loclx.io")
    window.tcp_client.connect_to_server()

    window.image_client = TcpImageClientWorker(host="ap.loclx.io", port=SEND_IMAGE_PORT)
    window.image_client.image_received.connect(window.handle_image_received)
    window.image_client.error_occurred.connect(lambda msg: print(f"[TCP-IMG 오류] {msg}"))
    window.image_client.start()

    window.show()
    sys.exit(app.exec_())
