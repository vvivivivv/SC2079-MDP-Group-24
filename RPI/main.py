from threading import Thread
from Android import AndroidInterface
from PC import PCInterface
from stm import STMInterface
from RPI.constants import STM_GYRO_RESET_COMMAND
import json

# Total Integration
class RPiMain:
    def __init__(self, task2):
        # Initialize interfaces
        self.Android = AndroidInterface(self)
        self.PC = PCInterface(self)
        self.STM = STMInterface(self, task2=task2)

    def connect_components(self):
        # Connect all components
        self.Android.connect()
        #self.PC.connect()
        self.STM.connect()

    def cleanup(self):
        # Disconnect from all components
        self.Android.disconnect()
        self.PC.disconnect()
        self.STM.disconnect()

    def forward_algo_path_to_android(self, path_data):
        """
        Forward the algo path JSON to Android.
        """
        message = {
            "type": "PATH",
            "data": path_data
        }
        if hasattr(self, "Android"):
            self.Android.msg_queue.put(json.dumps(message).encode("utf-8"))
            print("[RPiMain] Forwarded algo path to Android")

    def run(self):
        print("[RPiMain] Starting RPiMain...")

        # Connect components
        self.connect_components()
        print("[RPiMain] Components connected successfully")

        # Create threads for sending messages
        Android_send = Thread(target=self.Android.send, name="Android_send_thread")
        STM_send = Thread(target=self.STM.send, name="STM_send_thread")

        # Create threads for receiving messages
        Android_listen = Thread(target=self.Android.listen, name="Android_listen_thread")

        # Start sending threads
        Android_send.start()
        STM_send.start()
        print("[RPiMain] Sending threads started successfully")

        # Start listening threads
        Android_listen.start()
        print("[RPiMain] Listening threads started successfully")

        # Wait for threads to end
        Android_send.join()
        STM_send.join()
        Android_listen.join()

        print("[RPiMain] All threads concluded, cleaning up...")

        self.cleanup()

        print("[RPiMain] Exiting RPiMain...")

if __name__ == "__main__":
    rpi = RPiMain(True)
    try:
        rpi.run()
    except KeyboardInterrupt:
        print("[RPiMain] KeyboardInterrupt caught, cleaning up...")
        rpi.cleanup()
        print("[RPiMain] Cleanup complete, exiting.")