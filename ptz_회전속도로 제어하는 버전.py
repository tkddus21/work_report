#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custom_msgs.msg import WheelRPM
import socket
import threading
import subprocess
import json
import os
import time
import re
import urllib.request
from geometry_msgs.msg import Vector3
from visca_over_ip import Camera 


WATCH_FOLDER = '/home/railwayjetson1/Pictures/images'
SEND_IMAGE_PORT = 12100
CONTROL_PORT = 12000

def is_connected():
    try:
        urllib.request.urlopen('http://google.com', timeout=3)
        return True
    except:
        return False

def start_localxpose_tunnel(port: int):
    try:
        subprocess.Popen([
            "loclx", "tunnel", "-r", "tcp",
            "--reserved-endpoint", f"ap.loclx.io:{port}",
            "--to", f"127.0.0.1:{port}",
            "--region", "ap"
        ])
        rclpy.logging.get_logger("localxpose").info(f"🌐 LocalXpose 터널 열림: 외부포트={port} → localhost:{port}")
    except Exception as e:
        rclpy.logging.get_logger("localxpose").error(f"❌ LocalXpose 실행 실패: {e}")

class RosTcpServer(Node):
    def __init__(self):
        rclpy.logging.get_logger("RosTcpServer").info("🧪 RosTcpServer 생성 시작")
        super().__init__('ros_to_tcp_server')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.sock.bind(('0.0.0.0', CONTROL_PORT))
        self.sock.listen(1)
        self.get_logger().info(f"[{CONTROL_PORT}] 제어용 TCP 서버 대기 중...")

        self.conn = None
        self.recv_waiting_for = "header"
        self.recv_header = None

        self.create_subscription(Float64, '/total_distance', self.distance_callback, 10)
        self.create_subscription(WheelRPM, '/wheel_rpm', self.speed_callback, 10)
        self.create_subscription(Float64, '/left_rpm', self.speed_callback, 10)
        self.cmd_pub = self.create_publisher(WheelRPM, '/cmd_rpm', 10)
        

        self.camera = Camera("192.168.0.100", 52381)
        self.ptz_state_pub = self.create_publisher(Vector3, '/ptz_status', 10)
        self.ptz_state_timer = self.create_timer(3.0, self.query_camera_status)
        self.ptz_cmd_pub = self.create_publisher(Vector3, '/ptz_cmd', 10)
        self.ptz_publish_timer = self.create_timer(1.0, self.publish_ptz_command)

        

        self.send_lock = threading.Lock()

        threading.Thread(target=self.connection_loop, daemon=True).start()

        self.latest_distance = None
        self.distance_updated = False
        self.latest_speed = None
        self.speed_updated = False

        self.timer = self.create_timer(0.1, self.send_all_packets)

        # 서버 클래스 내부
        self.ptz_x = 0.0
        self.ptz_y = 0.0
        self.ptz_z = 0.0 #줌 값 


    def query_camera_status(self):
        try:
            pan, tilt = self.camera.get_pantilt_position()
            zoom = self.camera.get_zoom_position()

            # 초기값 저장
            self.ptz_x = pan
            self.ptz_y = tilt
            self.ptz_z = zoom
            self.get_logger().info(f"🧭 PTZ 초기값 설정: pan={pan}, tilt={tilt}, zoom={zoom}")

            msg = Vector3()
            msg.x = pan
            msg.y = tilt
            msg.z = zoom
            self.ptz_state_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"카메라 상태 조회 실패: {e}")


    def publish_ptz_command(self):
        msg = Vector3()
        msg.x = self.ptz_x
        msg.y = self.ptz_y
        msg.z = self.ptz_z
        self.ptz_cmd_pub.publish(msg)
        self.get_logger().info(f"📤 PTZ 명령 송신: pan={msg.x}, tilt={msg.y}, zoom={msg.z}")

    def connection_loop(self):
        while True:
            try:
                self.conn, addr = self.sock.accept()
                self.get_logger().info(f"[{CONTROL_PORT}] 클라이언트 객체 연결: {addr}")
                self.receive_loop()
            except Exception as e:
                self.get_logger().error(f"[ACCEPT ERROR] {e}")

    def calculate_checksum(self, data_str):
        return sum(bytearray(data_str.encode())) % 256

    def send_all_packets(self):
        if self.conn is None:
            return

        if self.distance_updated:
            self.send_packet("distance", {"value": self.latest_distance})
            self.distance_updated = False

        if self.speed_updated:
            left, right = self.latest_speed
            self.send_packet("wheel_rpm", {"left": left, "right": right})
            self.speed_updated = False

    def send_packet(self, type_str, body_dict):
        try:
            body_json = json.dumps(body_dict)
            header = {
                "type": type_str,
                "length": len(body_json.encode())
            }
            checksum = self.calculate_checksum(body_json)
            body_dict["checksum"] = checksum
            full_body_json = json.dumps(body_dict)

            with self.send_lock:
                self.conn.sendall((json.dumps(header) + '\n').encode())
                self.conn.sendall((full_body_json + '\n').encode())

            self.get_logger().info(f"[TX] {type_str} 전송 완료 | 길이={header['length']} | checksum={checksum}")
        except Exception as e:
            self.get_logger().error(f"[{type_str}] 전송 실패: {e}")

    def distance_callback(self, msg):
        self.latest_distance = msg.data
        self.distance_updated = True

    def speed_callback(self, msg):
        self.latest_speed = (msg.data, msg.data)
        self.speed_updated = True

    def receive_loop(self):
        buffer = ""
        while True:
            try:
                data = self.conn.recv(1024).decode()
                if not data:
                    self.get_logger().info("클라이언트 연결 종료 감지")
                    self.conn.close()
                    self.conn = None
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.handle_recv_line(line.strip())
            except Exception as e:
                self.get_logger().error(f"[RECV] 오류 : {e}")
                break

    def handle_recv_line(self, line):
        if self.recv_waiting_for == "header":
            try:
                self.recv_header = json.loads(line)
                self.recv_waiting_for = "body"
            except Exception as e:
                self.get_logger().error(f"[RECV] header parsing error: {e}")


        elif self.recv_waiting_for == "body":
            try:
                body = json.loads(line)
                checksum = body.pop("checksum", -1)
                body_str = json.dumps(body)
                expected = self.calculate_checksum(body_str)

                if checksum != expected:
                    self.get_logger().warn("checksum not matched!")
                else:
                    self.process_command(self.recv_header["type"], body)
                self.recv_waiting_for = "header"
            except Exception as e:
                self.get_logger().error(f"[RECV] body parsing error: {e}")
                self.recv_waiting_for = "header"

    def process_command(self, type_str, body):
        if type_str == "cmd_rpm":
            msg_rpm = WheelRPM()
            msg_rpm.left_rpm = body["left"]
            msg_rpm.right_rpm = body["right"]
            self.cmd_pub.publish(msg_rpm)
        
        
        elif type_str == "auto_drive_button":
            self.get_logger().info("🚗 자율주행 시작 명령 수신")

        elif type_str == "alarm_button":
            self.get_logger().info("🔔 알람 명령 수신")

        elif type_str == "return_button":
            self.get_logger().info("↩️ 복귀 명령 수신")

        elif type_str == "setting_button":
            self.get_logger().info("⚙️ 설정 명령 수신")


        # 버튼 입력이 왔을 때 처리 예시
        elif type_str == "left_button":
            action = body.get("value", "pressed")
            if action == "pressed":
                self.camera.pantilt(10, 5, relative=True)
                print("누름")
            elif action == "released":
                self.camera.pantilt(0, 0, relative=True)
                print("뗌")
                
        elif type_str == "right_button":
            action = body.get("value", "pressed")
            if action == "pressed":
                self.camera.pantilt(-10, 5, relative=True)
            elif action == "released":
                self.camera.pantilt(0, 0, relative=True)

        elif type_str == "up_button":
            self.ptz_y += 5.0  # 위로 5도 이동
            self.get_logger().info(f"현재 tilt 값: {self.ptz_y}")
        elif type_str == "down_button":
            self.ptz_y -= 5.0  # 아래로 5도 이동
            self.get_logger().info(f"현재 tilt 값: {self.ptz_y}")


        else:
            self.get_logger().warn(f"알 수 없는 수신 타입: {type_str}")
        
        self.camera.pantilt(0, 0)

class ImageTcpServer:
    def __init__(self):
        self.logger = rclpy.logging.get_logger("ImageTcpServer")
        self.logger.info("🧪 ImageTcpServer 생성 시작")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64*1024*1024)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64*1024*1024)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', SEND_IMAGE_PORT))
        self.sock.listen(1)
        self.logger.info(f"[{SEND_IMAGE_PORT}] 이미지 전송용 TCP 서버 대기 중...")

        self.conn = None
        self.sent_files = set()
        threading.Thread(target=self.connection_loop, daemon=True).start()

    def connection_loop(self):
        while True:
            try:
                self.conn, addr = self.sock.accept()
                self.logger.info(f"[{SEND_IMAGE_PORT}] 클라이언트 연결됨: {addr}")
                threading.Thread(target=self.watch_folder_loop, daemon=True).start()
            except Exception as e:
                self.logger.error(f"[{SEND_IMAGE_PORT}] 연결 실패: {e}")

    def is_file_stable(self, file_path, wait_time=1.0):
        try:
            size1 = os.path.getsize(file_path)
            time.sleep(wait_time)
            size2 = os.path.getsize(file_path)
            return size1 == size2
        except:
            return False

    def extract_number(self, filename):
        match = re.search(r'\d+', filename)
        return int(match.group()) if match else -1

    def watch_folder_loop(self):
        last_sent_time = {}
        while True:
            try:
                files = sorted(os.listdir(WATCH_FOLDER), key=self.extract_number)
                for file in files:
                    if file.endswith('.png'):
                        full_path = os.path.join(WATCH_FOLDER, file)
                        mod_time = os.path.getmtime(full_path)

                        if file not in last_sent_time or mod_time > last_sent_time[file]:
                            if self.is_file_stable(full_path):
                                self.send_image(full_path)
                                last_sent_time[file] = mod_time
                time.sleep(1)
            except Exception as e:
                self.logger.error(f"[Watcher] 폴더 감싸 실패: {e}")

    def send_image(self, image_path):
        try:
            if not self.conn:
                self.logger.warn("[IMG] 소스 연결 없음, 전송 스키프")
                return

            with open(image_path, 'rb') as f:
                img_data = f.read()
            size = len(img_data)
            self.logger.info(f"[SEND IMG] {image_path} 전송 시작")
            self.conn.sendall(size.to_bytes(4, byteorder='big'))
            self.conn.sendall(img_data)
            self.logger.info(f"[IMG] 전송 완료: {image_path} ({size} bytes)")

        except BrokenPipeError:
            self.logger.warn("[IMG] Broken pipe: 클라이언트가 연결을 끊었습니다.")
            self.conn.close()
            self.conn = None
        except Exception as e:
            self.logger.error(f"[IMG] 전송 실패: {e}")

def main(args=None):
    rclpy.init(args=args)

    if is_connected():
        start_localxpose_tunnel(CONTROL_PORT)
        start_localxpose_tunnel(SEND_IMAGE_PORT)
    else: 
        rclpy.logging.get_logger("localxpose").warn("🌐 네트워크가 없어 LocalXpose 생략됨, 로컬 통신만 활성화됩니다.")

    control_server = RosTcpServer()
    image_server = ImageTcpServer()

    try:
        rclpy.spin(control_server)
    except KeyboardInterrupt:
        pass
    finally:
        if control_server.conn:
            control_server.conn.close()
        control_server.sock.close()

        if image_server.conn:
            image_server.conn.close()
        image_server.sock.close()

        control_server.destroy_node()
        rclpy.shutdown()
        print("서버 종료")
