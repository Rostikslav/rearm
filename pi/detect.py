"""Red brick detection and px -> mm mapping."""

import math

import cv2
import numpy as np

import config


def _scaled_pins(size):
    """Pin coordinates rescaled from REFERENCE_SIZE to the given capture size."""
    fx = size[0] / config.REFERENCE_SIZE[0]
    fy = size[1] / config.REFERENCE_SIZE[1]
    return np.array([(x * fx, y * fy) for x, y in config.PINS_PX], dtype=np.float32)


def _scaled_center(size):
    fx = size[0] / config.REFERENCE_SIZE[0]
    fy = size[1] / config.REFERENCE_SIZE[1]
    cx, cy = config.OPTICAL_CENTER_PX
    return cx * fx, cy * fy


def build_transform(size):
    """Perspective map from capture pixels to millimetres in the arm's frame.

    Built from the pin TOPS, so it already contains the pins' own parallax.
    correct_parallax() below cancels that out for the brick.
    """
    src = _scaled_pins(size)
    dst = np.array(config.PINS_MM, dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def correct_parallax(points, size):
    """Pull points inward to undo the height difference between brick and pins.

    A feature at height h is displayed outward from the optical center by
    H / (H - h). The transform bakes in the pins' factor, so what remains for
    the brick is (H - brick) / (H - pin), applied about the optical center.
    """
    H = config.CAMERA_HEIGHT_MM
    k = (H - config.BRICK_HEIGHT_MM) / (H - config.PIN_HEIGHT_MM)

    cx, cy = _scaled_center(size)
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)

    out = np.empty_like(pts)
    out[:, 0] = cx + (pts[:, 0] - cx) * k
    out[:, 1] = cy + (pts[:, 1] - cy) * k
    return out


def to_mm(points, transform, size):
    """Parallax-correct pixel points and map them into millimetres"""
    pts = correct_parallax(points, size).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, transform).reshape(-1, 2)


def red_mask(image_bgr):
    """Binary mask of saturated red"""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    mask = None
    for lo, hi in config.HUE_BANDS:
        band = cv2.inRange(
            hsv,
            np.array([lo, config.SAT_MIN, config.VAL_MIN]),
            np.array([hi, 255, 255]),
        )
        mask = band if mask is None else cv2.bitwise_or(mask, band)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def detect(image_bgr, transform=None, debug=None):
    """Find the brick and return (x_mm, y_mm, w_mm, r_deg), or None.

    w_mm is the short side. r_deg is the long side's orientation in 0..179
    """
    size = (image_bgr.shape[1], image_bgr.shape[0])
    if transform is None:
        transform = build_transform(size)

    mask = red_mask(image_bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if debug is not None:
        debug["mask"] = mask
        debug["contours"] = len(contours)

    if not contours:
        if debug is not None:
            debug["error"] = "no red region found"
        return None

    # a single object is expected, only the biggest blob survives, the rest (if any) is noise
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    max_area = size[0] * size[1] * config.MAX_AREA_FRAC

    if debug is not None:
        debug["area"] = area

    if area < config.MIN_AREA_PX:
        if debug is not None:
            debug["error"] = f"largest red region too small ({area:.0f} px)"
        return None

    if area > max_area:
        if debug is not None:
            debug["error"] = f"largest red region too big ({area:.0f} px)"
        return None

    (cx, cy), (rw, rh), angle = cv2.minAreaRect(contour)

    # long axis in pixels
    if rw < rh:
        rw, rh = rh, rw
        angle += 90
    theta = math.radians(angle)
    half = rw / 2
    dx, dy = math.cos(theta) * half, math.sin(theta) * half

    # map the center and both axis endpoints together so the angle and the
    # width come out in millimetres rather than pixels
    probes = [
        (cx, cy),
        (cx - dx, cy - dy), (cx + dx, cy + dy),          # long axis
        (cx + dy * rh / rw, cy - dx * rh / rw),          # short axis, half step
    ]
    mm = to_mm(probes, transform, size)

    center = mm[0]
    long_vec = mm[2] - mm[1]
    short_half = mm[3] - mm[0]

    r_deg = math.degrees(math.atan2(long_vec[1], long_vec[0]))
    if config.ANGLE_COUNTERCLOCKWISE:
        r_deg = -r_deg
    r_deg = (r_deg + config.ANGLE_OFFSET_DEG) % 180
    w_mm = 2 * float(np.linalg.norm(short_half))

    if debug is not None:
        debug["rect_px"] = ((cx, cy), (rw, rh), angle)
        debug["error"] = None

    return (float(center[0]), float(center[1]), w_mm, r_deg)


def is_static(results):
    """True if every reading agrees within tolerance"""
    if len(results) < 2 or any(r is None for r in results):
        return False

    for a, b in zip(results, results[1:]):
        if abs(a[0] - b[0]) > config.STATIC_TOL_MM:
            return False
        if abs(a[1] - b[1]) > config.STATIC_TOL_MM:
            return False
        d = abs(a[3] - b[3]) % 180
        if min(d, 180 - d) > config.STATIC_TOL_DEG:
            return False

    return True
