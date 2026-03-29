from queue import Queue
import bluetooth as bt
import socket
import subprocess
import json
import time
import os

class AndroidInterface:
    def __init__(self, RPI):
        self.RPI = RPI
        self.uuid = "00001101-0000-1000-8000-00805F9B34FB" 
        self.msg_queue = Queue()
        self.socket = None 
        self.client_socket = None  

    def connect(self):
        print("[Android] Starting Bluetooth connection process...")

        # Aggressive cleanup: release any lingering RFCOMM processes/devices
        print("[Android] Cleaning up existing RFCOMM processes...")
        os.system("sudo pkill -9 rfcomm > /dev/null 2>&1")
        os.system("sudo rfcomm release all > /dev/null 2>&1")

        # Reset the Bluetooth adapter so stale state doesn't block binding
        print("[Android] Resetting Bluetooth interface (hci0)...")
        os.system("sudo hciconfig hci0 down")
        os.system("sudo hciconfig hci0 up")

        # Grant permission for Bluetooth access and make adapter discoverable
        subprocess.run("sudo chmod o+rw /var/run/sdp", shell=True)
        subprocess.run("sudo hciconfig hci0 piscan", shell=True)

        # Establish and bind socket
        self.socket = bt.BluetoothSocket(bt.RFCOMM)
        print("[Android] BT socket established successfully.")

        try:
            self.socket.bind(("", 1))  
            self.port = 1
            print("[Android] Bind to RFCOMM channel 1")
            self.socket.listen(128)
            bt.advertise_service(self.socket, "Group24", service_id=self.uuid,
                               service_classes=[self.uuid, bt.SERIAL_PORT_CLASS],
                               profiles=[bt.SERIAL_PORT_PROFILE])
            print("[Android] Waiting for Android connection...")
            time.sleep(2)  

            self.client_socket, self.client_info = self.socket.accept()
            print("[Android] Connected to", self.client_info)

        except Exception as e:
            print("[Android] Connection failed:", str(e))
            self.disconnect()  
            raise 

    def disconnect(self):
        try:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            if self.socket:
                bt.stop_advertising(self.socket)
                self.socket.close()
                self.socket = None
            print("[Android] Disconnected successfully.")
        except Exception as e:
            print("[Android] Failed to disconnect", str(e))

    def reconnect(self):
        self.disconnect()
        self.connect()

    def listen(self):
        buffer = b"" 
        last_received = time.time()
        while True:
            try:
                chunk = self.client_socket.recv(2048)
                if not chunk:
                    self.reconnect()
                    buffer = b""
                    last_received = time.time()
                    continue

                buffer += chunk
                last_received = time.time()

                if len(buffer) > 16384 :
                    print("[Android] ERROR: Buffer size exceeds maximum, clearing buffer")
                    buffer = b""
                    last_received = time.time()
                    continue

                try:
                    decodedMsg = buffer.decode("utf-8").strip()
                    if not decodedMsg:
                        buffer = b""
                        last_received = time.time()
                        continue

                    # Attempt JSON parse first
                    try:
                        parsedMsg = json.loads(decodedMsg)
                        msg_type = parsedMsg["type"]
                        print("[Android] JSON message from Android:", decodedMsg)
                    except json.JSONDecodeError:
                        # Fall back to plain text
                        msg_type = decodedMsg
                        parsedMsg = {"type": msg_type}
                        print("[Android] Plain-text message from Android:", decodedMsg)

                    # Route message
                    if msg_type == 'NAVIGATION':
                        self.RPiMain.STM.msg_queue.put(buffer)
                    elif msg_type in ('START_TASK','FASTEST_PATH','START_FASTEST'):
                        self.RPiMain.PC.handle_fastest_path(parsedMsg)
                        print("[Android] FASTEST_PATH/START_TASK:", decodedMsg)
                    else:
                        print("[Android] Error:", msg_type)

                    buffer = b""  
                    last_received = time.time()

                except UnicodeDecodeError as e:
                    if time.time() - last_received > 5:
                        print("[Android] Timeout Error")
                        buffer = b""
                        last_received = time.time()
                    continue  

            except (socket.error, IOError, ConnectionResetError) as e:
                print("[Android] ERROR:", str(e))
                self.reconnect()
                buffer = b""
                last_received = time.time()

    def send(self):
        while True: 
            message = self.msg_queue.get()
            exception = True
            while exception: 
                try:
                    self.client_socket.sendall(message)
                except Exception as e:
                    self.reconnect() 
                else:
                    exception = False  