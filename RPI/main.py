from threading import Thread
from Android import AndroidInterface
from PC import PCInterface
from stm import STMInterface
from upload_image import upload_images
import json

class RPITask:
    def __init__(self, task):
        self.Android = AndroidInterface(self)
        self.PC = PCInterface(self)
        self.STM = STMInterface(self, task=task)

    def connect(self):
        self.Android.connect()
        self.STM.connect()

    def disconnect(self):
        self.Android.disconnect()
        self.PC.disconnect()
        self.STM.disconnect()

    def send_algo(self, path):
        message = {
            "type": "PATH",
            "data": path
        }
        if hasattr(self, "Android"):
            self.Android.msg_queue.put(json.dumps(message).encode("utf-8"))
            print("[RPI] Forwarded algo path to Android")

    def run(self):
        print("[RPI] Starting RPI...")

        self.connect()
        print("[RPI] Components connected successfully")

        # Create threads 
        Android_send = Thread(target=self.Android.send, name="Android_send_thread")
        STM_send = Thread(target=self.STM.send, name="STM_send_thread")
        Android_listen = Thread(target=self.Android.listen, name="Android_listen_thread")

        # Start sending threads
        Android_send.start()
        STM_send.start()
        Android_listen.start()

        # Wait for threads to end
        Android_send.join()
        STM_send.join()
        Android_listen.join()

        print("[RPI] All threads concluded, cleaning up...")
        upload_images() 
        self.disconnect()

        print("[RPI] Exiting RPI...")

if __name__ == "__main__":
    rpi = RPITask(True)
    try:
        rpi.run()
    except KeyboardInterrupt:
        print("[RPI] KeyboardInterrupt")
        rpi.disconnect()
        print("[RPI] Exiting")