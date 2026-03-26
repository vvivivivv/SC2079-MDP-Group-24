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

# Initialize and start camera once at module load so it's ready when SNAP arrives
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
    """
    Capture image using the already-started Picamera2 and save it.

    Parameters:
        img_pth (str): Filename to save the captured image under FOLDER_PATH.
    """
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    camera.capture_file(image_save_location)
    print(f"[Camera] Image captured: {img_pth}")


def preprocess_img(img_pth: str) -> None:
    """
    Read image, resize it, and save the resized image.

    Parameters:
        img_pth (str): Filename of the image under FOLDER_PATH to preprocess.
    """
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    img = cv2.imread(image_save_location)

    resized_img = cv2.resize(img, (3000, 2464))  # (Width, Height)
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    cv2.imwrite(image_save_location, resized_img)
    print("[Camera] Image preprocessing complete")


def get_image(final_image: bool = False, obstacle_id: Optional[int] = None) -> bytes:
    """
    Capture an image, preprocess it, and return a JSON message with the encoded image.

    Returns:
        bytes: Encoded JSON message containing image data.
    """
    capture_time = time.time()
    timestamp = int(capture_time)

    if obstacle_id is not None:
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
        "type": "IMAGE_TAKEN",
        "final_image": final_image,
        "data": {
            "image": encoded_string,
            "filename": img_pth,
            "timestamp": timestamp
        }
    }
    return json.dumps(message).encode("utf-8")


def close_camera() -> None:
    """Call this on shutdown to cleanly release the camera."""
    camera.stop()
    camera.close()
    print("[Camera] Picamera2 closed.")