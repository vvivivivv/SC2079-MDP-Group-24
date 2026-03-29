import os
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = 'Task1Best_ncnn_model'
OUTPUT_DIR = '/home/mdpgrp24/yolo/images'

os.makedirs(OUTPUT_DIR, exist_ok=True)

print("[RPI_YOLO] Loading model...")
_model = YOLO(MODEL_PATH, task='detect')
print("[RPI_YOLO] Model ready.")


def infer(frame: np.ndarray, obstacle_id: str) -> str | None:
    results = _model(frame, conf=0.5)

    if len(results[0].boxes) == 0:
        print(f"[RPI_YOLO] No detection for obstacle {obstacle_id}.")
        _save_no_detect(frame, obstacle_id)
        return None

    boxes = results[0].boxes
    best_idx = boxes.conf.argmax()
    best_box = boxes[best_idx]

    new_conf = float(best_box.conf[0])
    class_index = int(best_box.cls[0])
    class_id = _model.names[class_index]

    print(f"[RPI_YOLO] Detected: class={class_id} conf={new_conf:.4f} obstacle={obstacle_id}")

    annotated_frame = results[0].plot()
    detection_filename = f"{obstacle_id}_detection_{class_id}_{new_conf:.4f}.jpg"
    detection_path = os.path.join(OUTPUT_DIR, detection_filename)

    no_detect_path = os.path.join(OUTPUT_DIR, f"{obstacle_id}_no_detect.jpg")
    if os.path.exists(no_detect_path):
        os.remove(no_detect_path)

    existing_file, existing_conf = _find_existing_detection(obstacle_id)

    if existing_file is None:
        cv2.imwrite(detection_path, annotated_frame)
        print(f"[RPI_YOLO] Saved {detection_filename}")
    elif new_conf > existing_conf:
        os.remove(os.path.join(OUTPUT_DIR, existing_file))
        cv2.imwrite(detection_path, annotated_frame)
        print(f"[RPI_YOLO] Replaced: {existing_conf:.4f} -> {new_conf:.4f}")
    else:
        print(f"[RPI_YOLO] Kept existing: {existing_conf:.4f} >= {new_conf:.4f}")
        class_id = existing_file.split('_detection_')[1].rsplit('_', 1)[0]

    return class_id


def _find_existing_detection(obstacle_id: str):
    for name in os.listdir(OUTPUT_DIR):
        if name.startswith(f"{obstacle_id}_detection_") and name.endswith(".jpg"):
            try:
                existing_conf = float(name.rsplit("_", 1)[-1].replace(".jpg", ""))
            except ValueError:
                existing_conf = -1.0
            return name, existing_conf
    return None, -1.0


def _save_no_detect(frame: np.ndarray, obstacle_id: str):
    detection_exists = any(
        name.startswith(f"{obstacle_id}_detection_") and name.endswith(".jpg")
        for name in os.listdir(OUTPUT_DIR)
    )
    if detection_exists:
        print(f"[RPI_YOLO] Detection already exists for obstacle {obstacle_id}. Skipping no-detect save.")
    else:
        cv2.imwrite(os.path.join(OUTPUT_DIR, f"{obstacle_id}_no_detect.jpg"), frame)
        print(f"[RPI_YOLO] Saved no_detect for obstacle {obstacle_id}")