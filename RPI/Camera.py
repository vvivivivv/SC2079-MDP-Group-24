import base64
import json
import os
from typing import Dict, Any, Optional
from picamera2 import Picamera2
import cv2
import time
from datetime import datetime

FOLDER_PATH = "/home/mdpgrp24/yolo/ImageCapture"
IMAGE_PREPROCESSED_FOLDER_PATH = "/home/mdpgrp24/yolo/ImagePreProcessed"

os.makedirs(FOLDER_PATH, exist_ok=True)
os.makedirs(IMAGE_PREPROCESSED_FOLDER_PATH, exist_ok=True)

# Initialize and start camera once at startup
print("[Camera] Initializing Picamera2...")
camera = Picamera2()
config = camera.create_preview_configuration(
    main={"format": "RGB888", "size": (3280, 2464)}
)
camera.configure(config)
camera.start()
time.sleep(2)  # Allow camera sensor to warm up
print("[Camera] Picamera2 ready.")


def capture(img_pth: str) -> None:
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    camera.capture_file(image_save_location)
    print(f"Image captured at {img_pth}")


def preprocess_img(img_pth: str) -> None:
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    img = cv2.imread(image_save_location)
    resized_img = cv2.resize(img, (3000, 2464)) 
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    cv2.imwrite(image_save_location, resized_img)


def get_image(image: bool = False, obstacle_id: Optional[int] = None) -> bytes:
    capture_time = time.time()
    timestamp = int(capture_time)

    if obstacle_id:
        img_pth = f"{timestamp}_{obstacle_id}.jpg"
    else:
        formatted_time = datetime.fromtimestamp(capture_time).strftime('%d-%m_%H-%M-%S.%f')[:-3]
        img_pth = f"img_{formatted_time}.jpg"
    capture(img_pth)
    preprocess_img(img_pth)
    encoded_string = ""
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    if os.path.isfile(image_save_location):
        with open(image_save_location, "rb") as img_file:
            encoded_string = base64.b64encode(img_file.read()).decode('utf-8')

    message: Dict[str, Any] = {
        "final_image": image,
        "data": {
            "image": encoded_string,
            "filename": img_pth,
            "timestamp": timestamp
        }
    }
    return json.dumps(message).encode("utf-8")


def close_camera() -> None:
    camera.stop()
    camera.close()
    print("[Camera] Picamera2 closed.")