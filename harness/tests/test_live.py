from __future__ import annotations

import io
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import pytest

from ringquad_harness.cli import main
from ringquad_harness.live import (
    LiveIngest, LiveStreamError, PoseBuffer, PoseSample, encode_record, read_records,
)
from ringquad_harness.schemas import Frame

NS = 1_000_000_000


def frame(capture_ns: int, clock: str = "sim_time", seq: int = 0) -> Frame:
    return Frame("s", "cam", seq, capture_ns, clock, np.zeros((4, 4, 3), np.uint8))


def pose(t: int, clock: str = "sim_time") -> PoseSample:
    return PoseSample(t, clock, "odom", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), "odom")


def jpeg(color: tuple[int, int, int], size=(32, 24)) -> bytes:
    img = np.full((size[1], size[0], 3), color, np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def test_exact_time_sample_matches() -> None:
    buf = PoseBuffer()
    buf.add(pose(5 * NS))
    assert buf.match(frame(5 * NS), max_age_ns=NS // 10).timestamp_ns == 5 * NS


def test_old_or_other_clock_sample_is_dropped() -> None:
    buf = PoseBuffer()
    buf.add(pose(5 * NS))
    assert buf.match(frame(5 * NS + NS // 2), max_age_ns=NS // 10) is None
    assert buf.match(frame(5 * NS, clock="host_wall"), max_age_ns=NS) is None


def test_sample_from_the_future_is_not_used() -> None:
    buf = PoseBuffer()
    buf.add(pose(6 * NS))
    assert buf.match(frame(5 * NS), max_age_ns=NS) is None


def test_newest_usable_sample_wins() -> None:
    buf = PoseBuffer()
    for t in (1, 2, 3):
        buf.add(pose(t * NS))
    assert buf.match(frame(3 * NS + 10), max_age_ns=NS).timestamp_ns == 3 * NS


def test_record_round_trip_and_truncation() -> None:
    blob = encode_record({"kind": "frame", "n": 1}, b"abc") + encode_record({"kind": "pose"})
    records = list(read_records(io.BytesIO(blob)))
    assert records == [({"kind": "frame", "n": 1}, b"abc"), ({"kind": "pose"}, b"")]
    with pytest.raises(LiveStreamError):
        list(read_records(io.BytesIO(blob[:-3])))


def test_record_rejects_oversized_lengths() -> None:
    bad = (2**31).to_bytes(4, "big") + (0).to_bytes(4, "big")
    with pytest.raises(LiveStreamError):
        list(read_records(io.BytesIO(bad)))


def frame_meta(seq: int, t: int, **extra) -> dict:
    return {"kind": "frame", "camera_id": "cam", "sequence": seq, "capture_ns": t,
            "clock_domain": "sim_time", "calibration_id": "cal0", **extra}


def pose_meta(t: int) -> dict:
    return {"kind": "pose", "timestamp_ns": t, "clock_domain": "sim_time", "frame_id": "odom",
            "position": [1, 2, 3], "orientation": [0, 0, 0, 1], "source": "odom"}


def test_missing_pose_still_yields_frame_without_world_target() -> None:
    ingest = LiveIngest("s", max_pose_age_ns=NS // 10)
    out = ingest.handle(frame_meta(0, NS), jpeg((0, 255, 0)))
    assert out.frame.capture_ns == NS and out.frame.calibration_id == "cal0"
    assert out.pose is None


def test_pose_joins_frame_and_goes_stale() -> None:
    ingest = LiveIngest("s", max_pose_age_ns=NS // 10)
    assert ingest.handle(pose_meta(NS), b"") is None
    assert ingest.handle(frame_meta(0, NS), jpeg((9, 9, 9))).pose is not None
    assert ingest.handle(frame_meta(1, 2 * NS), jpeg((9, 9, 9))).pose is None


def test_clock_reset_clears_pose_buffer() -> None:
    ingest = LiveIngest("s", max_pose_age_ns=10 * NS)
    ingest.handle(pose_meta(100 * NS), b"")
    ingest.handle(frame_meta(0, 100 * NS), jpeg((1, 1, 1)))
    out = ingest.handle(frame_meta(1, NS), jpeg((1, 1, 1)))
    assert out.clock_reset is True and out.pose is None


def test_corrupt_jpeg_is_rejected() -> None:
    ingest = LiveIngest("s", max_pose_age_ns=NS)
    with pytest.raises(LiveStreamError):
        ingest.handle(frame_meta(0, NS), b"not a jpeg")


class _Server:
    def __init__(self, blobs: list[bytes]):
        self.connections = 0
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                blob = blobs[min(outer.connections, len(blobs) - 1)]
                outer.connections += 1
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.end_headers()
                self.wfile.write(blob)

            def log_message(self, *_):
                pass

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.httpd.server_port}/stream"

    def close(self):
        self.httpd.shutdown()


def reddish() -> bytes:
    img = np.full((120, 160, 3), (60, 140, 50), np.uint8)
    img[40:90, 50:110] = (170, 120, 30)
    ok, buf = cv2.imencode(".jpg", img[:, :, ::-1])
    return buf.tobytes()


def test_live_cli_records_frames_pose_and_reconnects(tmp_path: Path) -> None:
    first = (encode_record({"kind": "status", "state": "mode=idle z=0.10"}) + encode_record(pose_meta(NS), b"")
             + encode_record(frame_meta(0, NS), reddish()))
    second = encode_record(frame_meta(1, 2 * NS), reddish())
    server = _Server([first, second])
    try:
        out = tmp_path / "live"
        code = main(["live", "--url", server.url, "--output", str(out), "--max-frames", "2",
                     "--reconnect-delay", "0.01", "--max-pose-age-ms", "100"])
    finally:
        server.close()
    assert code == 0
    manifest = json.loads((out / "manifest.json").read_text())
    stats = manifest["statistics"]
    assert stats["processed"] == 2 and stats["stale_pose"] == 1 and stats["reconnects"] >= 1
    events = [json.loads(line) for line in (out / "events.jsonl").read_text().splitlines()]
    frames = [e for e in events if e["event_type"] == "frame"]
    assert frames[0]["data"]["pose"]["frame_id"] == "odom"
    assert frames[1]["data"]["pose"] is None
    assert any(e["event_type"] == "reconnect" for e in events)
    assert [e["data"] for e in events if e["event_type"] == "status"] == [{"state": "mode=idle z=0.10"}]


def test_live_cli_fails_when_server_never_available(tmp_path: Path) -> None:
    code = main(["live", "--url", "http://127.0.0.1:9/stream", "--output", str(tmp_path / "o"),
                 "--max-frames", "1", "--reconnect-delay", "0.01", "--max-reconnects", "2"])
    assert code == 1
    assert not (tmp_path / "o" / "manifest.json").exists()
