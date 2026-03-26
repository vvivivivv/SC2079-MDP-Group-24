from queue import Queue
import bluetooth as bt
import socket
import sys
import subprocess
import json
import time
import os
from RPI.constants import *

class AndroidInterface:
    def __init__(self, RPiMain):
        self.RPiMain = RPiMain
        self.host = RPI_IP
        self.uuid = BT_UUID
        self.msg_queue = Queue()
        self.socket = None  # Initialize socket as None
        self.client_socket = None  # Initialize client_socket as None

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
            print("[Android] BT socket bound to RFCOMM channel 1")

            self.socket.listen(128)

            # Advertise Bluetooth service
            bt.advertise_service(self.socket, "Group24", service_id=self.uuid,
                               service_classes=[self.uuid, bt.SERIAL_PORT_CLASS],
                               profiles=[bt.SERIAL_PORT_PROFILE])
            print("[Android] Waiting for Android connection...")
            time.sleep(3)  # Delay for SDP propagation

            self.client_socket, self.client_info = self.socket.accept()
            print("[Android] Accepted connection from", self.client_info)

        except Exception as e:
            print("[Android] ERROR: connection failed -", str(e))
            self.disconnect()  # Clean up on failure
            raise  # Re-raise to allow caller to handle

    def disconnect(self):
        # Close the Bluetooth sockets
        try:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            if self.socket:
                bt.stop_advertising(self.socket)
                self.socket.close()
                self.socket = None
            print("[Android] Disconnected from Android successfully.")
        except Exception as e:
            print("[Android] ERROR: Failed to disconnect from Android -", str(e))

    def reconnect(self):
        # Disconnect and then connect again
        self.disconnect()
        self.connect()

    def listen(self):
        buffer = b""  # Buffer to accumulate incoming data
        last_received = time.time()
        MAX_BUFFER_SIZE = 16384 
        TIMEOUT = 5  
        while True:
            try:
                # Receive data from the Bluetooth socket
                chunk = self.client_socket.recv(BT_BUFFER_SIZE)
                if not chunk:
                    print("[Android] Android disconnected remotely. Reconnecting...")
                    self.reconnect()
                    buffer = b""
                    last_received = time.time()
                    continue

                buffer += chunk
                last_received = time.time()
                print("[Android] Received chunk of size:", len(chunk), "Total buffer size:", len(buffer))
                print("[Android] Buffer contents (hex):", buffer.hex()[:200])  

                if len(buffer) > MAX_BUFFER_SIZE:
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
                        print("[Android] WARNING: Unrecognised message type:", msg_type)

                    buffer = b""  
                    last_received = time.time()

                except UnicodeDecodeError as e:
                    if time.time() - last_received > TIMEOUT:
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
        # Continuously send messages to Android
        while True: 
            message = self.msg_queue.get()
            exception = True
            while exception: 
                try:
                    self.client_socket.sendall(message)
                    print("[Android] Write to Android: " + message.decode("utf-8")[:MSG_LOG_MAX_SIZE])
                except Exception as e:
                    print("[Android] ERROR: Failed to write to Android -", str(e))
                    self.reconnect() 
                else:
                    exception = False  
