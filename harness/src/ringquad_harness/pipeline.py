from __future__ import annotations

from pathlib import Path

from .perception import PerceptionConfig, detect_candidates
from .schemas import Frame, Observation
from .tracking import Tracker, TrackerConfig


class PerceptionPipeline:
    def __init__(self, evidence_dir: Path, *, perception: PerceptionConfig | None = None,
                 tracking: TrackerConfig | None = None):
        self.perception = perception or PerceptionConfig()
        self.tracker = Tracker(tracking or TrackerConfig(evidence_dir=evidence_dir))

    def update(self, frame: Frame) -> list[Observation]:
        return self.tracker.update(frame, detect_candidates(frame, self.perception))
