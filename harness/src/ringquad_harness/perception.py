from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .schemas import Candidate, Frame


@dataclass(frozen=True)
class PerceptionConfig:
    # Dark brown/black lesion-like pixels against green foliage. Deliberately
    # conservative: this is a proposal generator, not a disease classifier.
    min_area: int = 90
    min_extent_px: int = 16
    max_area_fraction: float = 0.18
    min_color_ratio: float = 0.32
    min_aspect: float = 0.38
    max_aspect: float = 2.8
    blur_kernel: int = 5


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
    max_area = int(height * width * config.max_area_fraction)
    candidates: list[Candidate] = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if min(w, h) < config.min_extent_px or area < config.min_area or area > max_area or x == 0 or y == 0 or x + w >= width or y + h >= height:
            continue
        aspect = w / max(h, 1)
        if not config.min_aspect <= aspect <= config.max_aspect:
            continue
        region = mask[y:y + h, x:x + w]
        ratio = float(np.count_nonzero(region)) / max(1, w * h)
        if ratio < config.min_color_ratio:
            continue
        # Relative compactness modestly favors coherent patches over long edges.
        score = min(1.0, ratio * (1.0 - abs(np.log(aspect)) * 0.15))
        candidates.append(Candidate((x, y, x + w, y + h), score, ratio))
    return sorted(candidates, key=lambda candidate: candidate.score, reverse=True)
