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


class SessionEvent(BaseModel):
    event_type: str
    timestamp_ns: int
    session_id: str
    camera_id: str | None = None
    sequence: int | None = None
    frame_id: str | None = None
    data: dict[str, Any] = Field(default_factory=dict)
