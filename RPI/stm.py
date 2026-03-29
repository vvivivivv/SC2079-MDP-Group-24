import json
from queue import Queue
import re
import time
import serial
import glob
from Camera import get_image
import time
from time import sleep

TURN_SLEEP = 2  # seconds to wait after each turn command
ROUTING_MAP = {
    "FIRSTLEFT": ["FL045", "FR045", "FR045", "FL045"],  
    "FIRSTRIGHT": ["FR045", "FL045", "FL045", "FR045"], 
    "SECONDLEFT": ["BW007", "FL090",  "IRR",  "FR090", "BW010", "FR090", "IRR", "BW015", "FR082", "FL005"],
    "SECONDRIGHT": ["BW010", "FR090", "IRL",  "FL090", "FL075", "IRL", "FL095"]
}
class STMInterface:
    def __init__(self, RPI, task):
        self.RPI = RPI
        self.baudrate = 115200
        self.serial = None
        self.msg_queue = Queue()
        self.task2 = task

    def connect(self):
        tty_ports = glob.glob('/dev/ttyACM*')
        if not tty_ports:
            print("[STM] ERROR: No ttyACM* ports found.")
            return

        for port in tty_ports:
            try:
                self.serial = serial.Serial(port, self.baudrate, write_timeout=0, timeout=5)
                print(f"[STM] Connected to STM successfully on {port}.")
                self.clean_buffers()
                return
            except Exception as e:
                print(f"[STM] Failed to connect to {port} - {str(e)}")
                continue
        print("[STM] ERROR: Failed to connect to any ttyACM* port.")
        self.serial = None
        if self.serial is None:
            print("[STM] WARNING: No serial connection established. Operations will fail.")

    def disconnect(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            print("[STM] Disconnected from STM.")

    def reconnect(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        self.connect()

    def clean_buffers(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

    def listen(self):
        message = None
        try:
            if self.serial is not None and self.serial.is_open:
                message = self.serial.read().decode("utf-8")
                print("[STM] Read from STM:", message[:150])
            else:
                print("[STM] Serial connection not available")
        except Exception as e:
            message = str(e)
            print("[STM] Listen error:", str(e))

        return message if message else None

    def wait_for_ack(self, timeout=5):
        start_time = time.time()
        while time.time() - start_time < timeout:
           message = self.listen()
           if message == 'A':
               print("[STM] Received ACK")
               return True
           time.sleep(0.1)
        print("[STM] No ACK received")
        return False

    def wait_for_dist(self, timeout=15.0, line_timeout=0.1, prefix="YF"):
        if self.serial is None or not self.serial.is_open:
            print("[STM] Serial connection not available")
            return -1

        pattern = re.compile(rf"{re.escape(prefix)}\s*(\d+)")
        end_time = time.time() + timeout
        old_timeout = getattr(self.serial, "timeout", None)
        self.serial.timeout = line_timeout
        try:
            buffer = ""
            while time.time() < end_time:
                try:
                    line = self.serial.readline().decode("utf-8", errors="replace")
                except AttributeError:
                    byte = self.serial.read(1)
                    if not byte:
                        time.sleep(0.01)
                        continue
                    line = byte.decode("utf-8", errors="replace")

                if not line:
                    time.sleep(0.01)
                    continue

                buffer += line
                m = pattern.search(buffer)
                if m:
                    raw_message = m.group(0)  
                    dist = int(m.group(1))
                    print(f"[STM] Received from STM: {raw_message}")
                    print(f"[STM] Received full distance: {dist}")
                    return dist

                if len(buffer) > 1024:
                    buffer = buffer[-512:]
            print(f"[STM] Timeout waiting for distance. Last buffer: '{buffer}'")
            return -1
        except serial.SerialException as e:
            print(f"[STM] Serial error: {e}")
            return -1
        finally:
            self.serial.timeout = old_timeout

    def wait_for_xdist(self, timeout=15.0, line_timeout=0.1, prefix="IR"):
        if self.serial is None or not self.serial.is_open:
            print("[STM] Error: Serial connection not available.")
            return -1

        pattern = re.compile(rf"{re.escape(prefix)}\s*(\d+)")
        end_time = time.time() + timeout

        old_timeout = getattr(self.serial, "timeout", None)
        self.serial.timeout = line_timeout

        try:
            buffer = ""
            while time.time() < end_time:
                try:
                    line = self.serial.readline().decode("utf-8", errors="replace")
                except AttributeError:
                    byte = self.serial.read(1)
                    if not byte:
                        time.sleep(0.01)
                        continue
                    line = byte.decode("utf-8", errors="replace")

                if not line:
                    time.sleep(0.01)
                    continue

                buffer += line
                match = pattern.search(buffer)
                if match:
                    raw_message = match.group(0)  
                    dist = int(match.group(1))
                    print(f"[STM] Received from STM: {raw_message}")
                    print(f"[STM] Received x distance: {dist}")
                    return dist

                if len(buffer) > 1024:
                    buffer = buffer[-512:]

            print(f"[STM] Timeout waiting for x distance. Last buffer: '{buffer}'")
            return -1

        except serial.SerialException as e:
            print(f"[STM] Serial error: {e}")
            return -1

        finally:
            self.serial.timeout = old_timeout

    def is_movement_command(self, command):
        return bool(re.match(r"^[FB][RL][0-9]{3}$", command))

    def send(self):
        while True:
            try:
                message_byte = self.msg_queue.get()
                message_str = message_byte.decode("utf-8")
                message = json.loads(message_str)
                message_type = message["type"]
            except Exception as e:
                print("[STM] Failed to process message:", str(e))
                self.reconnect()
                continue

            if message_type == "NAVIGATION":
                print("[STM] Sending cmds:", json.dumps(message, indent=2))
                self.send_path_to_android(message)
                commands = self.adjust_commands(message["data"]["commands"])
                i = 0
                while i < len(commands):
                    command = commands[i]
                    if command.startswith("SNAP"):
                        sleep(2)
                        obstacle_id = int(command[4:])
                        image_msg = get_image(final_image=False, obstacle_id=obstacle_id)
                        image_json = json.loads(image_msg.decode("utf-8"))
                        image_id = self.RPI.PC.get_arrow_direction(image_json)
                        if image_id is None:
                            i += 1
                            continue
                        is_left = image_id == "39"
                        prefix = "FIRST" if obstacle_id == 1 else "SECOND"
                        direction = prefix + "LEFT" if is_left else prefix + "RIGHT"
                        if prefix == "SECOND":
                            self.RPI.PC.second_arrow = 'L' if is_left else 'R'
                        adj_commands = self.adjust_commands([direction])
                        for adj_cmd in adj_commands:
                            print(f"[STM] Sending command to STM: {adj_cmd}")
                            self.write_to_stm(adj_cmd)
                            if self.is_movement_command(adj_cmd):
                                print(f"[STM] Sleeping {TURN_SLEEP}s after turn {adj_cmd}")
                                time.sleep(TURN_SLEEP)
                        i += 1  # Move past the SNAP to the next command
                        continue
                    elif command == "FIN":
                        self.RPI.PC.handle_fin()
                    else:
                        print(f"[STM] Sending command to STM: {command}")
                        self.write_to_stm(command)
                        if self.is_movement_command(command):
                            print(f"[STM] Sleeping {TURN_SLEEP}s after turn {command}")
                            time.sleep(TURN_SLEEP)
                    i += 1

            else:
                print("[STM] Unknown message type:", message_type)

    def send_path_to_android(self, message):
        # Forward path to Android if available
        if "path" in message["data"] and message["data"]["path"]:
            self.RPI.Android.msg_queue.put(self.create_path_message(message["data"]["path"]))

    def is_valid_command(self, command):
        return re.match('^(([FB][WS])|([UYV]F)|([FB][RL])|(IR[L|R]))[0-9]{0,3}$' , command)

    def is_obstacle_routing_command(self, command):
        return command in ROUTING_MAP

    def is_straight_command(self, command):
        return self.is_valid_command(command) and re.match("^[FB][WS][0-9]{3}$", command)

    def is_validturn_command(self, command):
        return self.is_valid_command(command) and (command.startswith("F") or command.startswith("B"))

    def combine_straight_commands(self, straight_commands):
        dir_dict = {"FW": 1, "BW": -1, "FS": 1, "BS": -1}
        total = 0
        is_slow = any(c.startswith("FS") or c.startswith("BS") for c in straight_commands)
        for c in straight_commands:
            dir = c[:2]
            val = int(c[2:])
            total += dir_dict.get(dir, 0) * val

        if total > 0:
            prefix = "FS" if is_slow else "FW"
            return f"{prefix}{abs(total):03d}"
        elif total < 0:
            prefix = "BS" if is_slow else "BW"
            return f"{prefix}{abs(total):03d}"
        else:
            return None

    def add_command(self, final, new):
        if self.is_straight_command(new) and (len(final) > 0 and self.is_straight_command(final[-1])):
            prev = final.pop(-1)
            combined = self.combine_straight_commands([prev, new])
            if combined is not None:
                final.append(combined)
            else:
                final.append(prev)
                final.append(new)
        else:
            final.append(new)
        return final

    def adjust_commands(self, commands):
        def adjust_obstacle_routing_command(command):
            # Adjust obstacle routing commands
            routing = ROUTING_MAP.get(command, [command])
            return routing

        final_commands = []
        for i in range(len(commands)):
            command = commands[i].upper()
            adj_commands = []
            if self.is_obstacle_routing_command(command):
                adj_commands = adjust_obstacle_routing_command(command)
            else:
                final_commands = self.add_command(final_commands, command)
            for c in adj_commands:
                final_commands = self.add_command(final_commands, c)
        return final_commands

    def create_path_message(self, path):
        message = {
            "type": "PATH",
            "data": {
                "path": path
            }
        }
        return json.dumps(message).encode("utf-8")

    def write_to_stm(self, command):
        if re.match("^YF[0-9]{3}$", command):
            self.clean_buffers() 
            self.serial.write((command + '\n').encode())
            dist = self.wait_for_dist()
            if dist >= 0:
                self.RPI.PC.handle_y_dist(dist)
            else:
                print(f"[STM] Invalid distance received for {command}: {dist}")
            return
        elif command in ["IRL", "IRR"]:
            self.clean_buffers()  
            self.serial.write((command + '\n').encode())
            dist = self.wait_for_xdist()
            if dist >= 0:
                self.RPI.PC.handle_x_dist(dist)
            else:
                print(f"[STM] Invalid x distance received for {command}: {dist}")
            return

        self.serial.write((command + '\n').encode())
        if not re.match("^YF[0-9]{3}$", command) and not re.match("^IR[0-9]{2}$", command):
            self.wait_for_ack()