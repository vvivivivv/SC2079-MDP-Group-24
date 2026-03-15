import os
import cv2
import math
import numpy as np

# ---- CONFIG ----
IMAGE_DIR = "received_images/"          # folder containing images
TARGET_W, TARGET_H = 320, 240            # size of each tile (W, H)
SPACING = 20                             # gap between tiles (px)
TOP_LABEL_PAD = 40                       # extra space above each tile for text (px)
WIN_NAME = "Run Summary"

VALID_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def load_images_sorted(folder: str):
    files = [f for f in os.listdir(folder) if f.lower().endswith(VALID_EXT)]
    files.sort()  # assumes filename ordering is the desired order

    images = []
    names = []
    for f in files:
        path = os.path.join(folder, f)
        img = cv2.imread(path)
        if img is None:
            continue
        img = cv2.resize(img, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
        images.append(img)
        names.append(f)

    return images, names


def draw_label(tile: np.ndarray, label: str) -> np.ndarray:
    """
    Create a tile with a top label band.
    """
    # Create white label band + image below
    tile_with_label = np.ones((TOP_LABEL_PAD + TARGET_H, TARGET_W, 3), dtype=np.uint8) * 255
    tile_with_label[TOP_LABEL_PAD:TOP_LABEL_PAD + TARGET_H, :, :] = tile

    # Put label text inside the top band
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 2

    (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
    x = max(10, (TARGET_W - text_w) // 2)
    y = max(text_h + 10, (TOP_LABEL_PAD + text_h) // 2)

    cv2.putText(tile_with_label, label, (x, y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return tile_with_label


def make_tiled_canvas(labeled_tiles: list[np.ndarray]) -> np.ndarray:
    """
    Tile up to 8 images. Layout: 4 columns x 2 rows (for max 8).
    If fewer than 8, it fills left-to-right, top-to-bottom.
    """
    n = len(labeled_tiles)
    if n == 0:
        raise ValueError("No images found in the folder.")

    cols = 4
    rows = math.ceil(n / cols)

    tile_h = TOP_LABEL_PAD + TARGET_H
    tile_w = TARGET_W

    canvas_h = rows * tile_h + (rows - 1) * SPACING
    canvas_w = cols * tile_w + (cols - 1) * SPACING

    # White background
    canvas = np.ones((canvas_h, canvas_w, 3), dtype=np.uint8) * 255

    for idx, tile in enumerate(labeled_tiles):
        r = idx // cols
        c = idx % cols

        y0 = r * (tile_h + SPACING)
        x0 = c * (tile_w + SPACING)

        canvas[y0:y0 + tile_h, x0:x0 + tile_w] = tile

    return canvas


def main():
    images, names = load_images_sorted(IMAGE_DIR)

    # Keep only the first 8 (your max)
    images = images[:8]
    names = names[:8]

    labeled_tiles = []
    for i, img in enumerate(images, start=1):
        # Label like "Image 1" (or include filename if you want)
        label = f"Obstacle {i}"
        labeled_tiles.append(draw_label(img, label))

    canvas = make_tiled_canvas(labeled_tiles)

    cv2.imshow(WIN_NAME, canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()