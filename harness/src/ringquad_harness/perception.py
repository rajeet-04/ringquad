from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .schemas import Candidate, Frame


@dataclass(frozen=True)
class PerceptionConfig:
    # Dark brown/black lesion-like pixels against green foliage. Deliberately
    # conservative: this is a proposal generator, not a disease classifier.
    # Pixel limits below are for a 480x320 frame and scale with frame area.
    reference_pixels: int = 480 * 320
    min_area: int = 90
    min_extent_px: int = 16
    max_area_fraction: float = 0.18
    min_color_ratio: float = 0.32
    min_aspect: float = 0.38
    max_aspect: float = 2.8
    blur_kernel: int = 5
    # A lesion sits on foliage: the ring around it must be at least this green.
    # Yellow-green counts (chlorotic halos); dark soil clods on bare soil have no such ring and are dropped.
    min_context_green: float = 0.25
    context_margin: float = 0.75


def detect_candidates(frame: Frame, config: PerceptionConfig | None = None) -> list[Candidate]:
    config = config or PerceptionConfig()
    image = frame.image
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Frame.image must be an RGB24 image")
    # Segment low-value, low-saturation lesion pixels in HSV. Median filtering
    # suppresses isolated sensor/compression glints.
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    kernel = max(3, config.blur_kernel | 1)
    hsv = cv2.medianBlur(hsv, kernel)
    mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([35, 255, 115]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    height, width = image.shape[:2]
    scale = max(1.0, height * width / config.reference_pixels)
    min_area = config.min_area * scale
    min_extent = config.min_extent_px * scale ** 0.5
    max_area = int(height * width * config.max_area_fraction)
    green = cv2.inRange(hsv, np.array([25, 40, 50]), np.array([90, 255, 255]))
    candidates: list[Candidate] = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if min(w, h) < min_extent or area < min_area or area > max_area or x == 0 or y == 0 or x + w >= width or y + h >= height:
            continue
        aspect = w / max(h, 1)
        if not config.min_aspect <= aspect <= config.max_aspect:
            continue
        region = mask[y:y + h, x:x + w]
        ratio = float(np.count_nonzero(region)) / max(1, w * h)
        if ratio < config.min_color_ratio:
            continue
        if _context_green(green, x, y, w, h, config.context_margin) < config.min_context_green:
            continue
        # Relative compactness modestly favors coherent patches over long edges.
        score = min(1.0, ratio * (1.0 - abs(np.log(aspect)) * 0.15))
        candidates.append(Candidate((x, y, x + w, y + h), score, ratio))
    return sorted(candidates, key=lambda candidate: candidate.score, reverse=True)


def _context_green(green: np.ndarray, x: int, y: int, w: int, h: int, margin: float) -> float:
    """Fraction of green pixels in the ring around a box (box grown by ``margin`` of its size)."""
    height, width = green.shape
    mx, my = max(2, int(w * margin)), max(2, int(h * margin))
    x0, y0, x1, y1 = max(0, x - mx), max(0, y - my), min(width, x + w + mx), min(height, y + h + my)
    outer = green[y0:y1, x0:x1]
    inner = green[y:y + h, x:x + w]
    ring = outer.size - inner.size
    return (np.count_nonzero(outer) - np.count_nonzero(inner)) / ring if ring > 0 else 0.0
