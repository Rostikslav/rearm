"""Vision unit HTTP server. Runs on the Pi; the ESP32 polls it.

  GET /ping    -> {"ok": true}
  GET /object  -> {"valid": true, "x":..,"y":..,"w":..,"r":..}
                  {"valid": false, "reason": "..."}

Also usable without the arm:
  python3 server.py --preview   save one annotated capture and exit
"""

import json
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

import config
import detect

_picam = None
_transform = None


def camera():
    global _picam
    if _picam is None:
        from picamera2 import Picamera2

        _picam = Picamera2()
        cfg = _picam.create_still_configuration(
            main={"size": config.CAPTURE_SIZE, "format": "RGB888"}
        )
        _picam.configure(cfg)
        _picam.start()
        time.sleep(2) # let auto exposure and white balance settle

        # lock exposure and white balance: drifting gains move the hue and
        # saturation of the brick between frames, which the static gate then
        # considers as movement
        meta = _picam.capture_metadata()
        _picam.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": int(meta["ExposureTime"]),
            "AnalogueGain": float(meta["AnalogueGain"]),
            "ColourGains": tuple(float(v) for v in meta["ColourGains"]),
        })
        time.sleep(0.5)

    return _picam


def capture():
    """Returns a BGR frame in the arm's orientation.

    The rotation happens here so that everything downstream is in the arm's point of view.
    """
    frame = camera().capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    return frame


def transform():
    global _transform
    if _transform is None:
        _transform = detect.build_transform(config.CAPTURE_SIZE)
    return _transform


def read_object(debug=None):
    """Capture several frames and return a reading only once it's settled."""
    results = []

    for i in range(config.STATIC_FRAMES):
        if i:
            time.sleep(config.STATIC_INTERVAL)
        results.append(detect.detect(capture(), transform(), debug=debug))

    if any(r is None for r in results):
        return None, "no object"

    if not detect.is_static(results):
        return None, "moving"

    # median of each field so that one bad frame cannot shift the answer
    return tuple(float(np.median([r[i] for r in results])) for i in range(4)), None


class Handler(BaseHTTPRequestHandler):
    def _send(self, payload):
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/ping"):
            self._send({"ok": True})
            return

        if not self.path.startswith("/object"):
            self.send_error(404)
            return

        try:
            result, reason = read_object()
        except Exception as exc:
            print("detection failed:", exc, flush=True)
            self._send({"valid": False, "reason": "error"})
            return

        if result is None:
            self._send({"valid": False, "reason": reason})
            return

        x, y, w, r = result
        payload = {
            "valid": True,
            "x": int(round(x)),
            "y": int(round(y)),
            "w": int(round(w)),
            "r": int(round(r)),
        }
        print("object:", payload, flush=True)
        self._send(payload)

    def log_message(self, *args):
        pass    # the prints above are enough


def preview():
    """Save one annotated capture (for checking calibration without the arm)."""
    image = capture()
    dbg = {}
    result = detect.detect(image, transform(), debug=dbg)

    cv2.imwrite("capture.jpg", image)
    if "mask" in dbg:
        cv2.imwrite("mask.jpg", dbg["mask"])

    if result is None:
        print("no detection:", dbg.get("error"))
        print("debug:", {k: v for k, v in dbg.items() if k != "mask"})
        return

    x, y, w, r = result
    print(f"x={x:.1f}mm  y={y:.1f}mm  w={w:.1f}mm  r={r:.1f}deg")

    box = cv2.boxPoints(dbg["rect_px"])
    cv2.drawContours(image, [np.int32(box)], 0, (0, 0, 255), 2)
    cv2.imwrite("detection.jpg", image)
    print("wrote capture.jpg, mask.jpg, detection.jpg")


if __name__ == "__main__":
    if "--preview" in sys.argv:
        preview()
        raise SystemExit(0)

    server = HTTPServer((config.HOST, config.PORT), Handler)
    print(f"vision unit listening on {config.HOST}:{config.PORT}", flush=True)
    server.serve_forever()
