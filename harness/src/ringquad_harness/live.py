"""Live ingest: framed records from the simulator bridge, joined to pose by capture time.

Wire format (one record): 4-byte big-endian JSON length, 4-byte big-endian payload length,
the JSON metadata, then the payload (a JPEG for ``kind == "frame"``, empty for ``"pose"``).
"""
from __future__ import annotations

import json
import struct
import time
from collections import deque
from dataclasses import dataclass
from typing import BinaryIO, Callable, Iterator

import cv2
import numpy as np

from .schemas import Frame

MAX_META_BYTES = 1 << 20
MAX_PAYLOAD_BYTES = 16 << 20


class LiveStreamError(ValueError):
    """The live stream is malformed or truncated."""


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    clock_domain: str
    frame_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    source: str

    def to_dict(self) -> dict:
        return {"timestamp_ns": self.timestamp_ns, "clock_domain": self.clock_domain,
                "frame_id": self.frame_id, "position": list(self.position),
                "orientation": list(self.orientation), "source": self.source}


class PoseBuffer:
    def __init__(self, maxlen: int = 512):
        self._samples: deque[PoseSample] = deque(maxlen=maxlen)

    def add(self, sample: PoseSample) -> None:
        self._samples.append(sample)

    def reset(self) -> None:
        self._samples.clear()

    def match(self, frame: Frame, max_age_ns: int) -> PoseSample | None:
        """Newest same-clock sample at or before the frame, no older than ``max_age_ns``."""
        best: PoseSample | None = None
        for sample in self._samples:
            if sample.clock_domain != frame.clock_domain or sample.timestamp_ns > frame.capture_ns:
                continue
            if best is None or sample.timestamp_ns > best.timestamp_ns:
                best = sample
        if best is None or frame.capture_ns - best.timestamp_ns > max_age_ns:
            return None
        return best


def encode_record(meta: dict, payload: bytes = b"") -> bytes:
    raw = json.dumps(meta, separators=(",", ":")).encode()
    return struct.pack(">II", len(raw), len(payload)) + raw + payload


def _read_exact(stream: BinaryIO, size: int, *, allow_eof: bool = False) -> bytes | None:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            if allow_eof and remaining == size:
                return None
            raise LiveStreamError("stream ended inside a record")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_records(stream: BinaryIO) -> Iterator[tuple[dict, bytes]]:
    while True:
        header = _read_exact(stream, 8, allow_eof=True)
        if header is None:
            return
        meta_len, payload_len = struct.unpack(">II", header)
        if meta_len > MAX_META_BYTES or payload_len > MAX_PAYLOAD_BYTES:
            raise LiveStreamError(f"record too large: meta={meta_len} payload={payload_len}")
        raw = _read_exact(stream, meta_len)
        payload = _read_exact(stream, payload_len) if payload_len else b""
        try:
            meta = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise LiveStreamError(f"invalid record metadata: {exc}") from exc
        if not isinstance(meta, dict):
            raise LiveStreamError("record metadata must be an object")
        yield meta, payload


@dataclass(frozen=True)
class LiveFrame:
    frame: Frame
    pose: PoseSample | None
    clock_reset: bool
    meta: dict


class LiveIngest:
    def __init__(self, session_id: str, *, max_pose_age_ns: int):
        self.session_id = session_id
        self.max_pose_age_ns = max_pose_age_ns
        self.poses = PoseBuffer()
        self._last_capture_ns: int | None = None

    def handle(self, meta: dict, payload: bytes) -> LiveFrame | None:
        kind = meta.get("kind")
        try:
            if kind == "pose":
                self.poses.add(PoseSample(
                    int(meta["timestamp_ns"]), str(meta["clock_domain"]), str(meta["frame_id"]),
                    tuple(float(v) for v in meta["position"]), tuple(float(v) for v in meta["orientation"]),
                    str(meta.get("source", "unknown"))))
                return None
            if kind != "frame":
                return None
            image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise LiveStreamError("frame payload is not a decodable image")
            capture_ns = int(meta["capture_ns"])
            frame = Frame(self.session_id, str(meta["camera_id"]), int(meta["sequence"]), capture_ns,
                          str(meta["clock_domain"]), image[:, :, ::-1].copy(), meta.get("calibration_id"))
        except (KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, LiveStreamError):
                raise
            raise LiveStreamError(f"invalid {kind} record: {exc!r}") from exc
        reset = self._last_capture_ns is not None and capture_ns < self._last_capture_ns
        if reset:
            self.poses.reset()
        self._last_capture_ns = capture_ns
        return LiveFrame(frame, self.poses.match(frame, self.max_pose_age_ns), reset, meta)


def iter_records(
    open_stream: Callable[[], BinaryIO], *, reconnect_delay: float = 1.0,
    max_reconnects: int | None = None,
) -> Iterator[tuple[str, tuple[dict, bytes] | str]]:
    """Yield ("record", (meta, payload)) and ("reconnect", reason); retry until the budget is spent."""
    failures = 0
    while True:
        try:
            with open_stream() as stream:
                for record in read_records(stream):
                    failures = 0
                    yield "record", record
            reason = "stream closed"
        except (OSError, LiveStreamError) as exc:
            reason = f"{type(exc).__name__}: {exc}"
        failures += 1
        if max_reconnects is not None and failures > max_reconnects:
            raise LiveStreamError(f"live stream unavailable after {max_reconnects} reconnects: {reason}")
        yield "reconnect", reason
        time.sleep(reconnect_delay)
