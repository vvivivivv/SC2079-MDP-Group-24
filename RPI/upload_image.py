import os
import requests

IMAGE_DIR = '/home/mdpgrp24/yolo/images'
SERVER_URL = 'http://192.168.24.226:5000/upload'


def upload_images():
    files = [f for f in os.listdir(IMAGE_DIR) if os.path.isfile(os.path.join(IMAGE_DIR, f))]
    if not files:
        print("[Upload] No images found to upload.")
        return

    print(f"[Upload] Uploading {len(files)} image(s) to {SERVER_URL}...")
    for filename in files:
        filepath = os.path.join(IMAGE_DIR, filename)
        try:
            with open(filepath, "rb") as f:
                resp = requests.post(SERVER_URL, files={"file": (filename, f, "image/jpeg")}, timeout=15)
            print(f"[Upload] {filename} -> {resp.status_code} {resp.text}")
        except Exception as e:
            print(f"[Upload] ERROR uploading {filename}: {e}")

    print("[Upload] Done.")