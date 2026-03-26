import json
import socket
import requests
import threading
import time
import base64
import cv2
import numpy as np
from queue import Queue
from RPI.constants import *
import rpi_yolo

# External services
YOLO_URL = "http://192.168.22.23:5000/imageyolo"
PC_TIMEOUT = 5  # seconds to wait for PC YOLO response

class PCInterface:
    def __init__(self, RPiMain):
        self.RPiMain = RPiMain
        self.msg_queue = Queue()
        self.client_socket = None
        self.send_message = False
        self.host = RPI_IP
        self.port = PC_PORT
        self.y_dists = []
        self.x_dists = []
        self.second_arrow = None
        self.obs_id = 1

    def connect(self):
        self.client_socket = None
        self.send_message = False
        print("[PC] Connection to server disabled (algo server not required).")

    def disconnect(self):
        if self.client_socket is not None:
            self.client_socket.close()
            self.client_socket = None
            self.send_message = False
            print("[PC] Disconnected from PC server.")

    def listen(self):
        while self.send_message:
            try:
                message = self.client_socket.recv(1024).decode("utf-8")
                if message:
                    print("[PC] Received from PC:", message)
                    parsed_msg = json.loads(message)
                    self.handle_message(parsed_msg)
            except Exception as e:
                print("[PC] ERROR: Failed to receive from PC -", e)
                self.disconnect()
                break

    def send(self):
        while self.send_message:
            try:
                message = self.msg_queue.get()
                if self.client_socket is not None:
                    self.client_socket.send(message.encode("utf-8"))
                    print("[PC] Sent to PC:", message)
            except Exception as e:
                print("[PC] ERROR: Failed to send to PC -", e)
                self.disconnect()
                break

    def handle_message(self, parsed_msg):
        message_type = parsed_msg["type"]
        if message_type == "FASTEST_PATH" or message_type == "START_TASK":
            self.handle_fastest_path(parsed_msg)
        elif message_type == "IMAGE":
            self.handle_image(parsed_msg)

    def handle_fastest_path(self, parsed_msg):
        try:
            commands = ["YF100", "SNAP1", "YF100", "SNAP2", "FIN"]
            resp = {
                "data": {
                    "commands": commands,
                    "path": []
                }
            }
            print("[PC] Generated Task 2 commands:", json.dumps(resp, indent=2))

            if resp["data"]["commands"] and hasattr(self.RPiMain, "STM"):
                nav_msg = {
                    "type": "NAVIGATION",
                    "data": {
                        "commands": resp["data"]["commands"],
                        "path": resp["data"]["path"]
                    }
                }
                self.RPiMain.STM.msg_queue.put(json.dumps(nav_msg).encode("utf-8"))

            if hasattr(self.RPiMain, "Android"):
                self.RPiMain.forward_algo_path_to_android(resp["data"]["path"])

            self.msg_queue.put(json.dumps(resp).encode("utf-8"))

        except Exception as e:
            print("[PC] ERROR handling FASTEST_PATH:", e)

    def handle_y_dist(self, dist):
        self.y_dists.append(dist)
        print(f"[PC] Stored y_dist: {dist}, Current y_dists: {self.y_dists}")

    def handle_x_dist(self, dist):
        self.x_dists.append(dist)
        print(f"[PC] Stored x_dist: {dist}, Current x_dists: {self.x_dists}")

    def get_arrow_direction(self, message):
        try:
            image_b64 = message["data"].get("image", "")
            if not image_b64:
                print("[PC] No image provided in message; skipping")
                return None

            img_bytes = base64.b64decode(image_b64)
            filename = message["data"].get("filename", "capture.jpg")
            obstacle_id = filename.replace('.jpg', '')  

            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                print("[PC] ERROR: Failed to decode image frame")
                return None
            pc_result = {}

            def call_pc_yolo():
                try:
                    yolo_filename = obstacle_id  
                    files = {"file": (yolo_filename, img_bytes, "image/jpeg")}
                    response = requests.post(YOLO_URL, files=files, timeout=PC_TIMEOUT).json()
                    print("[PC] YOLO server response:", json.dumps(response, indent=2))
                    pc_result['image_id'] = response.get("image_id")
                except requests.exceptions.Timeout:
                    print("[PC] WARNING: YOLO server timed out.")
                    pc_result['image_id'] = None
                    pc_result['timeout'] = True
                except Exception as e:
                    print(f"[PC] ERROR calling YOLO server: {e}")
                    pc_result['image_id'] = None

            pc_thread = threading.Thread(target=call_pc_yolo, daemon=True)
            pc_thread.start()


            print("[PC] Running RPI-side YOLO inference...")
            rpi_image_id = rpi_yolo.infer(frame, obstacle_id)
            print(f"[PC] RPI result: {rpi_image_id}")


            pc_thread.join(timeout=PC_TIMEOUT + 2)
            pc_image_id = pc_result.get('image_id')
            pc_timed_out = pc_result.get('timeout', False)


            if pc_timed_out or pc_image_id is None:
                print(f"[PC] PC YOLO unavailable. Using RPI result: {rpi_image_id}")
                return rpi_image_id

            if pc_image_id == rpi_image_id:
                print(f"[PC] Both agree: {rpi_image_id}")
                return rpi_image_id

            # Different results
            print(
                f"[PC] ERROR: RPI and PC disagree — RPI={rpi_image_id}, PC={pc_image_id}. "
                f"Using RPI result."
            )
            return rpi_image_id

        except Exception as e:
            print("[PC] ERROR in get_arrow_direction:", e)
            return None

    def handle_image(self, message):
        pass

    def handle_fin(self):
        if self.y_dists and self.x_dists and self.second_arrow:
            print(f"[PC] Initiating return to carpark with y_dists: {self.y_dists}, x_dists: {self.x_dists}, second_arrow: {self.second_arrow}")
            commands = self.get_commands_to_carpark()
            if commands:
                nav_msg = {"type": "NAVIGATION", "data": {"commands": commands, "path": []}}
                self.RPiMain.STM.msg_queue.put(json.dumps(nav_msg).encode("utf-8"))
                print("[PC] Sent return commands to STM:", commands)
            else:
                print("[PC] ERROR: No valid return commands generated.")
        else:
            print("[PC] ERROR: Missing y_dists or x_dists or second_arrow for return to carpark.")

    def get_commands_to_carpark(self):
        if not self.y_dists or not self.x_dists or not self.second_arrow:
            return []

        movement_list = []
        total_y_dist = sum(self.y_dists)
        if self.second_arrow == 'L':
            y_adjustment = total_y_dist + 5
            x_adjustment = self.x_dists[1] - 55
        elif self.second_arrow == 'R':
            y_adjustment = total_y_dist + 5
            x_adjustment = self.x_dists[1] - 59
        else:
            print("[PC] ERROR: Invalid second_arrow -", self.second_arrow)
            return []

        if self.second_arrow == 'R':
            movement_list.append(f"FW{y_adjustment:03d}")
            movement_list.append("FL090")
            if x_adjustment < 0:
                x_adjustment = -x_adjustment
                x_adjustment += 0
                movement_list.append(f"BW{x_adjustment:03d}")
            else:
                movement_list.append(f"FW{x_adjustment:03d}")
            movement_list.append("FR090")
            movement_list.append("YF100")
        elif self.second_arrow == 'L':
            movement_list.append(f"FW{y_adjustment:03d}")
            movement_list.append("FR090")
            if x_adjustment < 0:
                x_adjustment = -x_adjustment
                movement_list.append(f"BW{x_adjustment:03d}")
            else:
                movement_list.append(f"FW{x_adjustment:03d}")
            movement_list.append("FL090")
            movement_list.append("YF100")

        return movement_list
