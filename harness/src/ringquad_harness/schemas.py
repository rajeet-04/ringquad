from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from pydantic import BaseModel, Field


@dataclass(frozen=True)
class Frame:
    session_id: str
    camera_id: str
    sequence: int
    capture_ns: int
    clock_domain: str
    image: np.ndarray
    calibration_id: str | None = None

    @property
    def frame_id(self) -> str:
        return f"{self.session_id}:{self.camera_id}:{self.sequence}"


@dataclass(frozen=True)
class Candidate:
    bbox: tuple[int, int, int, int]
    score: float
    color_ratio: float = 0.0


@dataclass(frozen=True)
class EvidenceFrame:
    frame_id: str
    capture_ns: int
    context_path: str
    crop_path: str


@dataclass(frozen=True)
class Observation:
    observation_id: str
    session_id: str
    camera_id: str
    first_capture_ns: int
    last_capture_ns: int
    bbox: tuple[int, int, int, int]
    evidence_frame_ids: tuple[str, ...]
    evidence: tuple[EvidenceFrame, ...] = ()
    context_path: str | None = None
    crop_path: str | None = None
    status: str = "reviewable"


class SessionEvent(BaseModel):
    event_type: str
    timestamp_ns: int
    session_id: str
    camera_id: str | None = None
    sequence: int | None = None
    frame_id: str | None = None
    data: dict[str, Any] = Field(default_factory=dict)
