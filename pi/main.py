import json
import math
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from picamera2 import Picamera2

from object_detection import (
    DETECTION_VERSION,
    background_quality,
    detect_object,
    save_debug_images,
)

BACKGROUND_PATH = Path("background.jpg")
CAMERA_SETTINGS_PATH = Path("camera_settings.json")


def _load_camera_settings() -> Optional[dict]:
    if not CAMERA_SETTINGS_PATH.exists():
        return None
    settings = json.loads(CAMERA_SETTINGS_PATH.read_text())
    if "ColourGains" in settings:
        settings["ColourGains"] = tuple(float(v) for v in settings["ColourGains"])
    return settings


def _save_camera_settings(settings: dict) -> None:
    CAMERA_SETTINGS_PATH.write_text(json.dumps(settings))


def capture_area_image() -> np.ndarray:
    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start()

    saved = _load_camera_settings()
    if saved is not None:
        picam2.set_controls(saved)
        time.sleep(0.5)
    else:
        time.sleep(2)
        metadata = picam2.capture_metadata()
        settings = {
            "AeEnable": False,
            "ExposureTime": int(metadata["ExposureTime"]),
            "AnalogueGain": float(metadata["AnalogueGain"]),
            "AwbEnable": False,
            "ColourGains": tuple(float(v) for v in metadata["ColourGains"]),
        }
        picam2.set_controls(settings)
        time.sleep(0.5)
        _save_camera_settings(settings)

    image = picam2.capture_array()
    picam2.stop()
    picam2.close() # <--- RAFAEL ADD THIS LINE
    return image


def rotate(image: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate a numpy image by angle_deg and crop away the black/empty borders."""
    h, w = image.shape[:2]
    center = (w / 2, h / 2)

    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    cos = abs(M[0, 0])
    sin = abs(M[0, 1])

    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    M[0, 2] += (new_w / 2) - center[0]
    M[1, 2] += (new_h / 2) - center[1]

    rotated = cv2.warpAffine(
        image,
        M,
        (new_w, new_h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )

    angle = math.radians(abs(angle_deg))
    sin_a = abs(math.sin(angle))
    cos_a = abs(math.cos(angle))

    if h <= 0 or w <= 0:
        return rotated

    crop_w = int(w * cos_a - h * sin_a)
    crop_h = int(h * cos_a - w * sin_a)

    if crop_w <= 0 or crop_h <= 0:
        return rotated

    cx, cy = new_w // 2, new_h // 2

    x1 = max(cx - crop_w // 2, 0)
    y1 = max(cy - crop_h // 2, 0)
    x2 = min(cx + crop_w // 2, new_w)
    y2 = min(cy + crop_h // 2, new_h)

    return rotated[y1:y2, x1:x2]


def prepare_workspace_image() -> np.ndarray:
    """Capture from Pi camera, align, crop workspace, return BGR for OpenCV."""
    img = capture_area_image()
    img = rotate(img, 1)
    img = img[50 : img.shape[0] - 150, :]
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def save_detection_preview(image_bgr: np.ndarray, result) -> np.ndarray:
    overlay = image_bgr.copy()
    overlay[result.mask > 0] = (0, 200, 0)
    preview = cv2.addWeighted(image_bgr, 0.6, overlay, 0.4, 0)

    box_points = cv2.boxPoints(((result.x, result.y), (result.W, result.L), result.deg))
    cv2.drawContours(preview, [np.int32(box_points)], 0, (0, 0, 255), 2)
    return preview


def recalibrate() -> None:
    print(f"Detection version: {DETECTION_VERSION}", flush=True)
    print("IMPORTANT: remove ALL objects from the sheet before continuing.")

    if BACKGROUND_PATH.exists():
        BACKGROUND_PATH.unlink()
    if CAMERA_SETTINGS_PATH.exists():
        CAMERA_SETTINGS_PATH.unlink()

    input("Press Enter when the sheet is empty...")

    img = prepare_workspace_image()
    cv2.imwrite("area.jpg", img)
    cv2.imwrite(str(BACKGROUND_PATH), img)

    quality = background_quality(img, img)
    print(f"Saved {BACKGROUND_PATH}")
    print(f"Saved {CAMERA_SETTINGS_PATH}")
    print(f"Self-check (should be ~0): {quality}")
    print("Place the object on the sheet, then run: python main.py")


def test_background() -> None:
    print(f"Detection version: {DETECTION_VERSION}", flush=True)

    if not BACKGROUND_PATH.exists():
        print(f"missing {BACKGROUND_PATH} - run: python main.py --recalibrate")
        raise SystemExit(1)

    background = cv2.imread(str(BACKGROUND_PATH))
    if background is None:
        print(f"couldn't load {BACKGROUND_PATH}")
        raise SystemExit(1)

    print("Capture an EMPTY sheet to verify background.jpg...")
    input("Press Enter when the sheet is empty...")

    img = prepare_workspace_image()
    quality = background_quality(img, background)
    save_debug_images(img, background, prefix="test")
    print("quality:", quality)
    print("saved test_diff.jpg and test_mask.jpg")
    if quality["foreground_frac"] > 0.05:
        print("FAIL: background does not match the camera view - run --recalibrate")
    else:
        print("OK: background matches the empty sheet")


if __name__ == "__main__":
    if "--recalibrate" in sys.argv:
        recalibrate()
        raise SystemExit(0)

    if "--test-background" in sys.argv:
        test_background()
        raise SystemExit(0)

    print(f"Detection version: {DETECTION_VERSION}", flush=True)

    import serial

    controller_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    print("Waiting for EV Shield to send 'REQ'...", flush=True)

    while True:
        try:  # <-- 8 spaces
            if controller_serial.in_waiting > 0:
                incoming_request = controller_serial.readline().decode('utf-8').rstrip()

                if incoming_request == "REQ":
                    print("Request received! Taking picture...", flush=True)

                    img = prepare_workspace_image()
                    cv2.imwrite("area.jpg", img)

                    if not BACKGROUND_PATH.exists():
                        print(f"Missing {BACKGROUND_PATH}. Run: python main.py --recalibrate")
                        raise SystemExit(1)

                    background = cv2.imread(str(BACKGROUND_PATH))
                    if background is None:
                        print(f"couldn't load {BACKGROUND_PATH}")
                        raise SystemExit(1)

                    if background.shape[:2] != img.shape[:2]:
                        print("image size changed - run: python main.py --recalibrate")
                        raise SystemExit(1)

                    debug: dict = {}
                    result = detect_object(img, background_image=background, debug=debug)
                    save_debug_images(img, background)

                    print("debug:", debug, flush=True)

                    if result:
                        print(result)
                        cv2.imwrite("detection.jpg", save_detection_preview(img, result))
                        cv2.imwrite("mask.jpg", result.mask)

                        FIXED_MM_PER_PIXEL = 0.0819
                        x_mm = int(result.x * FIXED_MM_PER_PIXEL)
                        y_mm = int(result.y * FIXED_MM_PER_PIXEL)
                        w_mm = int(result.W * FIXED_MM_PER_PIXEL)
                        h_mm = int(result.L * FIXED_MM_PER_PIXEL)
                        r_deg = int(result.deg)

                        try: #--Add Rafael, 31 JUly--
                            # 1. Send coordinates to EV Shield 
                            data_string = f"{x_mm},{y_mm},{w_mm},{h_mm},{r_deg}\n"
                            controller_serial.write(data_string.encode('utf-8'))
                            print(f"Successfully sent via USB: {data_string.strip()}")
                            
                            # 2. PAUSE & WAIT FOR USER KEYPRESS
                            input("\n>>> [PAUSED] Press ENTER in terminal to move the arm...")

                            # 3. Send "GO" command to EV Shield
                            controller_serial.write(b"GO\n")
                            print("Sent 'GO' command! Arm starting movement...\n", flush=True)
                            
                        except Exception as e:
                            print(f"Failed to send data via USB. Error: {e}")

                    else:
                        print("detection failed")
                        if debug.get("error"):
                            print(debug["error"])
                        print("next steps:")
                        print("  1. python main.py --recalibrate   (empty sheet only)")
                        print("  2. python main.py --test-background")
                        print("  3. check debug_diff.jpg and debug_mask.jpg on the Pi")

        except (serial.SerialException, OSError) as e:  # <-- 8 spaces (matches `try:`)
            print(f"Serial connection lost (ESP32 likely rebooted): {e}. Reconnecting...")
            time.sleep(2)
            try:
                controller_serial.close()
            except Exception:
                pass

            # Try to reconnect indefinitely until the ESP32 comes back online
            while True:
                try:
                    controller_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
                    print("Reconnected to EV Shield! Resuming loop...")
                    break
                except Exception:
                    time.sleep(2)
