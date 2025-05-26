#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custom_msgs.msg import WheelRPM
import socket
import threading
import subprocess
import json
import struct
import os
import time
import re
import urllib.request
from geometry_msgs.msg import Vector3
from visca_over_ip import Camera 
from std_msgs.msg import Int32
import numpy as np
import pickle


WATCH_FOLDER = '/home/railwayjetson1/Pictures/line_camera_images'
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
        super().__init__('ros_to_tcp_server')
        self.get_logger().info("🧪 RosTcpServer 생성 시작")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', CONTROL_PORT))
        self.sock.listen(1)
        self.get_logger().info(f"[{CONTROL_PORT}] 제어용 TCP 서버 대기 중...")

        self.conn = None
        self.send_lock = threading.Lock()

        self.create_subscription(Float64, '/total_distance', self.distance_callback, 10)
        self.create_subscription(WheelRPM, '/wheel_rpm', self.speed_callback, 10)
        self.create_subscription(Float64, '/left_rpm', self.speed_callback, 10)
        self.cmd_pub = self.create_publisher(WheelRPM, '/cmd_rpm', 10)

        self.camera = Camera("192.168.88.100", 52381)
        self.ptz_state_pub = self.create_publisher(Vector3, '/ptz_status', 10)
        # self.ptz_state_timer = self.create_timer(3.0, self.query_camera_status)
        self.ptz_cmd_pub = self.create_publisher(Vector3, '/ptz_cmd', 10)
        # self.ptz_publish_timer = self.create_timer(1.0, self.publish_ptz_command)
        self.ptz_cmd_rotate_pub = self.create_publisher(Int32, '/ptz_cmd_rotate', 10)

        threading.Thread(target=self.connection_loop, daemon=True).start()

        self.latest_distance = None
        self.distance_updated = False
        self.latest_speed = None
        self.speed_updated = False

        self.timer = self.create_timer(0.1, self.send_all_packets)



        ## dict 
        self.keys = None
        self.values = None

        self.direction_map = {
                "left_button": 1,
                "right_button": -1,
                "up_button": 2,
                "down_button": -2
            }

        # msg data
        self.msg_cam = Int32()
        self.msg_rpm = WheelRPM()



    def recv_exactly(self, size):
        data = b''
        while len(data) < size:
            more = self.conn.recv(size - len(data))
            if not more:
                raise ConnectionError("연결 끊김")
            data += more
        return data

    def connection_loop(self):
        while True:
            try:
                self.conn, addr = self.sock.accept()
                self.get_logger().info(f"[{CONTROL_PORT}] 클라이언트 연결: {addr}")
                self.receive_loop()
            except Exception as e:
                self.get_logger().error(f"[ACCEPT ERROR] {e}")

    def receive_loop(self):
        while True:
            # try:
            # 1. 4바이트 길이 수신
            len_bytes = self.recv_exactly(4)
            msg_length = struct.unpack("!I", len_bytes)[0]

            # 2. JSON 본문 수신
            # json_bytes = self.recv_exactly(msg_length)
            # json_str = json_bytes.decode()
            data = self.recv_exactly(msg_length)
            message = pickle.loads(data)



            # # 3. 파싱 및 처리
            # data = json.loads(json_str)
            # self.process_command(data.get("type"), data)

            self.process_command("control_state", message)
            # except Exception as e:
            #     self.get_logger().error(f"[RECV 오류] {e}")
            #     if self.conn:
            #         self.conn.close()
            #         self.conn = None
            #     break

    def send_packet(self, type_str, body_dict):
        try:
            body_dict["type"] = type_str
            json_bytes = json.dumps(body_dict).encode()
            header = struct.pack("!I", len(json_bytes))

            with self.send_lock:
                self.conn.sendall(header + json_bytes)

            self.get_logger().info(f"[TX] {type_str} 전송 완료 ({len(json_bytes)} bytes)")
        except Exception as e:
            self.get_logger().error(f"[TX {type_str} 오류] {e}")

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

    def distance_callback(self, msg):
        self.latest_distance = msg.data
        self.distance_updated = True

    def speed_callback(self, msg):
        self.latest_speed = (msg.data, msg.data)
        self.speed_updated = True


    def send_dictdata(self, dict_cmd, bool_list):

        for idx, is_changed in enumerate(bool_list):
            # 값의 변경이 있는 경우
            if is_changed:
                target_key = self.keys[idx]
                print(target_key)

                if target_key == "rpm":
                    self.msg_rpm.left_rpm = dict_cmd["rpm"]
                    self.msg_rpm.right_rpm = dict_cmd["rpm"]
                    self.cmd_pub.publish(self.msg_rpm)

                elif target_key == "zoom":
                    pass


                elif target_key in ["auto_drive_button", "alarm_button", "return_button", "setting_button"]:
                    pass


                elif target_key in [ "left_button", "right_button", "up_button", "down_button"]:
                    self.msg_cam.data = int((dict_cmd[target_key] == "pressed") * self.direction_map[target_key])
                    self.ptz_cmd_rotate_pub.publish(self.msg_cam)



    def process_command(self, type_str, body):
        if type_str == "control_state":
            # self.get_logger().info(f"[CONTROL_STATE 수신] {body}")
            # dict keys : rpm, zoom, auto_drive_button, alarm_button, return_button, setting_button, up_button, down_button, right_button, left_button

            if self.keys == None:
                self.keys = list(body.keys())

            _values = np.array(list(body.values()))

            # 초기상태 모든 값 업데이트
            if self.values is None:
                self.values = _values
                bool_list = [True]*len(_values)
                self.send_dictdata(body, bool_list)

                
            # 초기화 후 수정된 내용만 업데이트
            else:
                bool_list = (self.values != _values).tolist()
                self.send_dictdata(body, bool_list)
                self.values = _values




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
        self.start_index = 0  # GUI로부터 받은 시작 인덱스를 저장할 변수

        threading.Thread(target=self.connection_loop, daemon=True).start()

    def connection_loop(self):
        while True:
            try:
                self.conn, addr = self.sock.accept()
                self.logger.info(f"[{SEND_IMAGE_PORT}] 클라이언트 연결됨: {addr}")


                 # 🎯 GUI가 보낸 start_index 받기!
                header = self.conn.recv(4)
                if not header:
                    continue
                size = int.from_bytes(header, byteorder='big')
                index_bytes = self.conn.recv(size)
                self.start_index = int.from_bytes(index_bytes, byteorder='big')
                self.logger.info(f"[🎯 시작 인덱스 수신 완료] {self.start_index}부터 이미지 전송 시작합니다!")

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
                    if file.endswith('.jpg'):

                        file_idx = self.extract_number(file)
                        if file_idx < self.start_index:
                            continue
                          
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
