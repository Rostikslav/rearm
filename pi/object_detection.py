# Frame-difference object detection with oriented bounding box.
# Compares the current frame to a saved empty-sheet background.jpg.
# Uses dual-threshold diffing to separate object from shadow.

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional

DETECTION_VERSION = "7-dual-threshold"

__all__ = [
    "DETECTION_VERSION",
    "DetectionResult",
    "is_static",
    "calibrate_background",
    "background_quality",
    "detect_object",
    "save_debug_images",
]

_DEFAULT_DIFF_THRESHOLD = 15   # catches object + shadow
_OBJECT_DIFF_THRESHOLD  = 90 # catches object only (shadow is softer, misses this)
_MIN_CONTOUR_AREA = 200
_MAX_CONTOUR_AREA_FRAC = 0.15
_MIN_CONTOUR_AREA_FRAC = 0.0003
_BORDER_MARGIN = 10
_MAX_FOREGROUND_FRAC = 0.25


@dataclass
class DetectionResult:
    x: float
    y: float
    W: float
    L: float
    deg: float
    confidence: float
    label: str
    mask: np.ndarray

    def __repr__(self):
        return (
            f"DetectionResult(x={self.x:.1f}, y={self.y:.1f}, "
            f"W={self.W:.1f}, L={self.L:.1f}, deg={self.deg:.1f}°, "
            f"conf={self.confidence:.2f}, label='{self.label}')"
        )


def is_static(
    results: list[Optional["DetectionResult"]],
    tolerance_px: float = 10.0,
    tolerance_deg: float = 2.0,
) -> bool:
    if len(results) < 2:
        return False

    for a, b in zip(results, results[1:]):
        if a is None or b is None:
            return False
        if abs(a.x - b.x) > tolerance_px:
            return False
        if abs(a.y - b.y) > tolerance_px:
            return False
        angle_diff = abs(a.deg - b.deg)
        angle_diff = min(angle_diff, 180 - angle_diff)
        if angle_diff > tolerance_deg:
            return False

    return True


def calibrate_background(
    image: np.ndarray,
    bounds: Optional[tuple[int, int, int, int]] = None,
    corner_frac: float = 0.08,
) -> tuple[float, float, float]:
    roi, _, _ = _crop_roi(image, bounds)
    h, w = roi.shape[:2]
    patch_w = max(1, int(w * corner_frac))
    patch_h = max(1, int(h * corner_frac))

    corners = [
        roi[0:patch_h, 0:patch_w],
        roi[0:patch_h, w - patch_w:w],
        roi[h - patch_h:h, 0:patch_w],
        roi[h - patch_h:h, w - patch_w:w],
    ]

    lab_samples = []
    for patch in corners:
        lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).astype(np.float32)
        lab_samples.append(lab.reshape(-1, 3))

    stacked = np.vstack(lab_samples)
    return tuple(np.median(stacked, axis=0).tolist())


def _crop_roi(
    image: np.ndarray,
    bounds: Optional[tuple[int, int, int, int]],
) -> tuple[np.ndarray, int, int]:
    h_img, w_img = image.shape[:2]
    if bounds is None:
        return image, 0, 0

    bx, by, bw, bh = bounds
    bx = max(0, min(bx, w_img))
    by = max(0, min(by, h_img))
    bw = max(1, min(bw, w_img - bx))
    bh = max(1, min(bh, h_img - by))
    return image[by:by + bh, bx:bx + bw], bx, by


def _resize_to_match(image: np.ndarray, reference: np.ndarray) -> np.ndarray:
    if image.shape[:2] == reference.shape[:2]:
        return image
    return cv2.resize(
        image,
        (reference.shape[1], reference.shape[0]),
        interpolation=cv2.INTER_LINEAR,
    )


def _match_brightness(image_bgr: np.ndarray, reference_bgr: np.ndarray) -> np.ndarray:
    image_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    ref_gray = cv2.cvtColor(reference_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    image_mean = float(image_gray.mean())
    ref_mean = float(ref_gray.mean())
    if image_mean < 1.0:
        return image_bgr

    scale = ref_mean / image_mean
    return np.clip(image_bgr.astype(np.float32) * scale, 0, 255).astype(np.uint8)


def _frame_diff_map(image_bgr: np.ndarray, background_bgr: np.ndarray) -> np.ndarray:
    background_bgr = _resize_to_match(background_bgr, image_bgr)
    image_bgr = _match_brightness(image_bgr, background_bgr)

    diff = cv2.absdiff(image_bgr, background_bgr)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(diff_gray, (5, 5), 0)


def _mask_from_diff(diff: np.ndarray, diff_threshold: float) -> np.ndarray:
    _, mask = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def _object_mask_from_dual_threshold(
    diff: np.ndarray,
    low_threshold: float,
    high_threshold: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Run the diff at two thresholds:
      low  → object + shadow region
      high → object only (shadow is too faint to pass this)

    If the high-threshold mask has enough content, use it as the object mask.
    If the object is also faint (e.g. a light-colored object), fall back to
    the low-threshold mask — it's still better than nothing.

    Returns (object_mask, shadow_mask) where shadow_mask = low - high.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    _, low_raw = cv2.threshold(diff, low_threshold, 255, cv2.THRESH_BINARY)
    low_mask = cv2.morphologyEx(low_raw, cv2.MORPH_OPEN,  kernel, iterations=1)
    low_mask = cv2.morphologyEx(low_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    _, high_raw = cv2.threshold(diff, high_threshold, 255, cv2.THRESH_BINARY)
    high_mask = cv2.morphologyEx(high_raw, cv2.MORPH_OPEN,  kernel, iterations=1)
    high_mask = cv2.morphologyEx(high_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    high_pixels = int(np.count_nonzero(high_mask))
    low_pixels  = int(np.count_nonzero(low_mask))

    # if the high threshold found at least 20% of what the low threshold found,
    # trust it as the object — the rest is shadow
    if high_pixels > low_pixels * 0.20:
        object_mask = high_mask
    else:
        # object is faint — fall back to low threshold
        object_mask = low_mask

    # shadow = whatever the low threshold found that the object mask didn't
    shadow_mask = cv2.bitwise_and(low_mask, cv2.bitwise_not(object_mask))

    return object_mask, shadow_mask


def _border_touch_count(contour: np.ndarray, width: int, height: int, margin: int) -> int:
    x, y, bw, bh = cv2.boundingRect(contour)
    touches = 0
    if x <= margin:
        touches += 1
    if y <= margin:
        touches += 1
    if x + bw >= width - margin:
        touches += 1
    if y + bh >= height - margin:
        touches += 1
    return touches


def _best_contour_from_mask(
    mask: np.ndarray,
    diff: np.ndarray,
    min_contour_area: int,
    max_contour_area_frac: float,
    min_contour_area_frac: float,
) -> Optional[np.ndarray]:
    """
    Collect all mask fragments, merge their points, return the convex hull.
    This handles thin objects (like a pen) that get split into fragments.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    height, width = mask.shape[:2]
    roi_area = height * width
    max_area = roi_area * max_contour_area_frac
    min_area = max(min_contour_area, roi_area * min_contour_area_frac)

    best_score = -1.0
    all_points = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area * 0.1:
            continue
        if _border_touch_count(contour, width, height, _BORDER_MARGIN) >= 3:
            continue

        contour_mask = np.zeros_like(mask)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
        mean_diff = float(diff[contour_mask > 0].mean()) if np.any(contour_mask > 0) else 0.0
        score = mean_diff * np.sqrt(area)

        if score > best_score * 0.3:
            all_points.append(contour.reshape(-1, 2))
            if score > best_score:
                best_score = score

    if not all_points:
        return None

    merged = np.vstack(all_points)
    hull = cv2.convexHull(merged)

    hull_area = cv2.contourArea(hull)
    if hull_area < min_area or hull_area > max_area:
        return None

    return hull


def _normalize_rect(w: float, h: float, angle: float) -> tuple[float, float, float]:
    angle_deg = float(angle) % 180
    if w < h:
        w, h = h, w
        angle_deg = (angle_deg + 90) % 180
    return w, h, angle_deg


def background_quality(
    image: np.ndarray,
    background_image: np.ndarray,
    diff_threshold: float = _DEFAULT_DIFF_THRESHOLD,
) -> dict:
    roi, _, _ = _crop_roi(image, None)
    bg_roi = _resize_to_match(_crop_roi(background_image, None)[0], roi)
    diff = _frame_diff_map(roi, bg_roi)
    mask = _mask_from_diff(diff, diff_threshold)

    return {
        "mean_diff": float(diff.mean()),
        "median_diff": float(np.median(diff)),
        "foreground_frac": float(np.count_nonzero(mask) / mask.size),
    }


def detect_object(
    image: np.ndarray,
    bounds: Optional[tuple[int, int, int, int]] = None,
    bg_lab: Optional[tuple[float, float, float]] = None,
    background_image: Optional[np.ndarray] = None,
    diff_threshold: float = _DEFAULT_DIFF_THRESHOLD,
    object_diff_threshold: float = _OBJECT_DIFF_THRESHOLD,
    min_contour_area: int = _MIN_CONTOUR_AREA,
    max_contour_area_frac: float = _MAX_CONTOUR_AREA_FRAC,
    min_contour_area_frac: float = _MIN_CONTOUR_AREA_FRAC,
    debug: Optional[dict] = None,
) -> Optional[DetectionResult]:
    """
    Detect one object by differencing against background_image (empty sheet).

    diff_threshold        — low threshold, catches object + shadow (default 15)
    object_diff_threshold — high threshold, catches object only   (default 40)
    """
    if background_image is None:
        return None

    roi, offset_x, offset_y = _crop_roi(image, bounds)
    bg_roi, _, _ = _crop_roi(background_image, bounds)
    bg_roi = _resize_to_match(bg_roi, roi)

    diff_roi = _frame_diff_map(roi, bg_roi)

    # low threshold mask for foreground fraction check
    low_mask = _mask_from_diff(diff_roi, diff_threshold)
    foreground_frac = float(np.count_nonzero(low_mask) / low_mask.size)

    if debug is not None:
        debug["mean_diff"] = float(diff_roi.mean())
        debug["median_diff"] = float(np.median(diff_roi))
        debug["foreground_frac"] = foreground_frac
        debug["diff_threshold"] = diff_threshold
        debug["object_diff_threshold"] = object_diff_threshold

    if foreground_frac > _MAX_FOREGROUND_FRAC:
        if debug is not None:
            debug["error"] = (
                "too much of the frame looks changed - background.jpg was probably "
                "saved with an object on the sheet, or lighting moved a lot"
            )
        return None

    # dual-threshold split: object vs shadow
    object_mask, shadow_mask = _object_mask_from_dual_threshold(
        diff_roi, diff_threshold, object_diff_threshold
    )

    if debug is not None:
        debug["object_mask_frac"] = float(np.count_nonzero(object_mask) / object_mask.size)
        debug["shadow_mask_frac"] = float(np.count_nonzero(shadow_mask) / shadow_mask.size)

    contour = _best_contour_from_mask(
        object_mask,
        diff_roi,
        min_contour_area,
        max_contour_area_frac,
        min_contour_area_frac,
    )

    if contour is None:
        if debug is not None:
            debug["error"] = (
                "no object contour found - try lowering diff_threshold or "
                "object_diff_threshold. "
                f"(low mask covers {foreground_frac:.1%} of frame)"
            )
        return None

    final_mask = np.zeros_like(object_mask)
    cv2.drawContours(final_mask, [contour], -1, 255, thickness=cv2.FILLED)

    rect = cv2.minAreaRect(contour)
    (cx_roi, cy_roi), (w, h), angle = rect
    w, h, angle_deg = _normalize_rect(w, h, angle)

    h_img, w_img = image.shape[:2]
    full_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    full_mask[offset_y:offset_y + roi.shape[0], offset_x:offset_x + roi.shape[1]] = final_mask

    if debug is not None:
        debug["contour_area_frac"] = float(cv2.contourArea(contour) / object_mask.size)
        debug["error"] = None

    roi_area = roi.shape[0] * roi.shape[1]
    fg_pixels = int(cv2.contourArea(contour))
    confidence = min(1.0, fg_pixels / max(roi_area * 0.003, 1))

    return DetectionResult(
        x=float(cx_roi) + offset_x,
        y=float(cy_roi) + offset_y,
        W=float(w),
        L=float(h),
        deg=angle_deg,
        confidence=confidence,
        label="object",
        mask=full_mask,
    )


def save_debug_images(
    image: np.ndarray,
    background_image: np.ndarray,
    diff_threshold: float = _DEFAULT_DIFF_THRESHOLD,
    object_diff_threshold: float = _OBJECT_DIFF_THRESHOLD,
    prefix: str = "debug",
) -> None:
    roi, _, _ = _crop_roi(image, None)
    bg_roi = _resize_to_match(_crop_roi(background_image, None)[0], roi)
    diff = _frame_diff_map(roi, bg_roi)

    low_mask = _mask_from_diff(diff, diff_threshold)
    object_mask, shadow_mask = _object_mask_from_dual_threshold(
        diff, diff_threshold, object_diff_threshold
    )

    cv2.imwrite(f"{prefix}_diff.jpg", diff)
    cv2.imwrite(f"{prefix}_mask.jpg", low_mask)        # full region (object + shadow)
    cv2.imwrite(f"{prefix}_object.jpg", object_mask)   # object only
    cv2.imwrite(f"{prefix}_shadow.jpg", shadow_mask)   # shadow only


if __name__ == "__main__":
    import sys

    path = sys.argv[1] if len(sys.argv) > 1 else "area.jpg"
    bg_path = sys.argv[2] if len(sys.argv) > 2 else "background.jpg"

    img = cv2.imread(path)
    background = cv2.imread(bg_path)
    if img is None or background is None:
        print("usage: python object_detection.py area.jpg background.jpg")
        sys.exit(1)

    print(DETECTION_VERSION, flush=True)
    print("quality:", background_quality(img, background))
    save_debug_images(img, background)

    debug: dict = {}
    result = detect_object(img, background_image=background, debug=debug)
    print("debug:", debug)

    if result:
        print(result)
        overlay = img.copy()
        overlay[result.mask > 0] = (0, 200, 0)
        preview = cv2.addWeighted(img, 0.6, overlay, 0.4, 0)
        box_points = cv2.boxPoints(((result.x, result.y), (result.W, result.L), result.deg))
        cv2.drawContours(preview, [np.int32(box_points)], 0, (0, 0, 255), 2)
        cv2.imwrite("detection.jpg", preview)
    else:
        print("nothing detected")
