from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence
import uuid

import cv2
import numpy as np

from .schemas import Candidate, EvidenceFrame, Frame, Observation


@dataclass(frozen=True)
class TrackerConfig:
    min_views: int = 3
    max_gap_ns: int = 2_000_000_000
    max_centroid_distance_fraction: float = 0.20
    max_tracks: int = 64
    evidence_dir: Path | None = None


@dataclass
class _Track:
    observation_id: str
    first_ns: int
    last_ns: int
    bbox: tuple[int, int, int, int]
    views: int = 0
    missed: int = 0
    reviewed: bool = False
    evidence: list[EvidenceFrame] = field(default_factory=list)
    pending: list[tuple[Frame, Candidate, float]] = field(default_factory=list)
    candidate: Candidate | None = None


def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return inter / max(1, area_a + area_b - inter)


def _center(box: tuple[int, int, int, int]) -> tuple[float, float]:
    return ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)


class Tracker:
    """Greedy temporal tracker with global image translation compensation.

    A phase-correlation estimate aligns consecutive frames before IoU and
    centroid association, tolerating the smooth camera pan/zoom in replay clips.
    One brief missing sample is retained and an emitted track is never re-emitted.
    """

    def __init__(self, config: TrackerConfig | None = None):
        self.config = config or TrackerConfig()
        self._tracks: list[_Track] = []
        self._previous_gray: np.ndarray | None = None
        self._sequence = 0

    def update(self, frame: Frame, candidates: Sequence[Candidate]) -> list[Observation]:
        gray = cv2.cvtColor(frame.image, cv2.COLOR_RGB2GRAY)
        shift = (0.0, 0.0)
        if self._previous_gray is not None and self._previous_gray.shape == gray.shape:
            (dx, dy), response = cv2.phaseCorrelate(self._previous_gray.astype(np.float32), gray.astype(np.float32))
            if response > 0.02 and np.isfinite(dx + dy):
                shift = (dx, dy)
        self._previous_gray = gray
        # Expire by capture time. Capture timestamps, not wall-clock arrival, govern gaps.
        self._tracks = [t for t in self._tracks if frame.capture_ns - t.last_ns <= self.config.max_gap_ns]
        unmatched = set(range(len(candidates)))
        pairs: list[tuple[float, int, int]] = []
        h, w = gray.shape
        max_distance = max(w, h) * self.config.max_centroid_distance_fraction
        for ti, track in enumerate(self._tracks):
            translated = (int(round(track.bbox[0] + shift[0])), int(round(track.bbox[1] + shift[1])),
                          int(round(track.bbox[2] + shift[0])), int(round(track.bbox[3] + shift[1])))
            for ci, candidate in enumerate(candidates):
                distance = np.linalg.norm(np.subtract(_center(translated), _center(candidate.bbox)))
                overlap = _iou(translated, candidate.bbox)
                if distance <= max_distance or overlap >= 0.01:
                    pairs.append((overlap + max(0.0, 1 - distance / max_distance) * 0.3, ti, ci))
        used_tracks: set[int] = set()
        used_candidates: set[int] = set()
        for _, ti, ci in sorted(pairs, reverse=True):
            if ti in used_tracks or ci in used_candidates:
                continue
            used_tracks.add(ti)
            used_candidates.add(ci)
            unmatched.discard(ci)
            track, candidate = self._tracks[ti], candidates[ci]
            track.bbox = candidate.bbox
            track.last_ns = frame.capture_ns
            track.views += 1
            track.missed = 0
            track.candidate = candidate
            track.pending.append((frame, candidate, self._sharpness(frame, candidate)))
        for ti, track in enumerate(self._tracks):
            if ti not in used_tracks:
                track.missed += 1
        for ci in sorted(unmatched):
            if len(self._tracks) >= self.config.max_tracks:
                break
            candidate = candidates[ci]
            track = _Track(str(uuid.uuid4()), frame.capture_ns, frame.capture_ns, candidate.bbox, views=1, candidate=candidate)
            track.pending.append((frame, candidate, self._sharpness(frame, candidate)))
            self._tracks.append(track)
        ready: list[Observation] = []
        for track in self._tracks:
            if track.reviewed or track.views < self.config.min_views:
                continue
            track.reviewed = True
            # Save at most three sharp views, avoiding per-frame disk churn. Keep
            # temporal order after selecting sharp frames for useful context.
            selected = sorted(sorted(track.pending, key=lambda item: item[2], reverse=True)[:3],
                              key=lambda item: item[0].capture_ns)
            track.evidence = [ev for ev in (self._save_evidence(f, c, track.observation_id) for f, c, _ in selected) if ev]
            ids = tuple(item.frame_id for item in track.evidence)
            latest = track.evidence[-1] if track.evidence else None
            ready.append(Observation(track.observation_id, frame.session_id, frame.camera_id,
                                     track.first_ns, track.last_ns, track.bbox, ids, tuple(track.evidence),
                                     latest.context_path if latest else None,
                                     latest.crop_path if latest else None))
        self._sequence += 1
        return ready

    @staticmethod
    def _sharpness(frame: Frame, candidate: Candidate) -> float:
        x1, y1, x2, y2 = candidate.bbox
        crop = frame.image[max(0, y1):min(frame.image.shape[0], y2), max(0, x1):min(frame.image.shape[1], x2)]
        if crop.size == 0:
            return 0.0
        return float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY), cv2.CV_64F).var())

    def _save_evidence(self, frame: Frame, candidate: Candidate, track_id: str) -> EvidenceFrame | None:
        root = self.config.evidence_dir
        if root is None:
            return None
        root.mkdir(parents=True, exist_ok=True)
        x1, y1, x2, y2 = candidate.bbox
        height, width = frame.image.shape[:2]
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        name = f"{track_id[:8]}-{frame.sequence:06d}"
        context = root / f"{name}-context.jpg"
        crop = root / f"{name}-crop.jpg"
        if not cv2.imwrite(str(context), cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise OSError(f"failed to write context evidence: {context}")
        if not cv2.imwrite(str(crop), cv2.cvtColor(frame.image[y1:y2, x1:x2], cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 96]):
            raise OSError(f"failed to write crop evidence: {crop}")
        return EvidenceFrame(frame.frame_id, frame.capture_ns, str(context), str(crop))
