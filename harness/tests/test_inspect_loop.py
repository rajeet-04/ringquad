from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from ringquad_harness.cli import main
from ringquad_harness.inspect_loop import InspectionError, build_request, load_inspectable

POSE = {"timestamp_ns": 5, "clock_domain": "sim_time", "frame_id": "odom",
        "position": [0, 0, 1], "orientation": [0, 0, 0, 1], "source": "odom"}
CAL = {"k": [400, 0, 320, 0, 400, 240, 0, 0, 1], "width": 640, "height": 480}


def write_session(root: Path, *, outcome="inspect", pose=POSE, calibration=CAL) -> Path:
    root.mkdir()
    events = [
        {"event_type": "calibration", "timestamp_ns": 1, "session_id": "s",
         "data": {"calibration_id": "cal0", "calibration": calibration}},
        {"event_type": "frame", "timestamp_ns": 5, "session_id": "s", "frame_id": "s:cam:3",
         "data": {"calibration_id": "cal0", "pose": pose}},
        {"event_type": "observation", "timestamp_ns": 5, "session_id": "s", "frame_id": "s:cam:3",
         "data": {"observation_id": "o1", "bbox": [10, 20, 50, 60], "pose": pose, "world_target": None}},
        {"event_type": "review", "timestamp_ns": 9, "session_id": "s",
         "data": {"observation_id": "o1", "status": "completed",
                  "decision": {"observation_id": "o1", "outcome": outcome, "evidence_frame_ids": ["s:cam:3"],
                               "explanation": "x", "requested_view": "closer" if outcome == "inspect" else None}}},
    ]
    (root / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events) + "\n")
    return root


def test_request_is_built_from_the_reviewed_observation(tmp_path: Path) -> None:
    request = build_request(load_inspectable(write_session(tmp_path / "s")))
    assert request["observation_id"] == "o1" and request["evidence_ids"] == ["s:cam:3"]
    assert request["bbox"] == [10, 20, 50, 60] and request["pose"]["frame_id"] == "odom"
    assert request["calibration"]["width"] == 640 and request["requested_view"] == "closer"


def test_only_inspect_decisions_are_inspectable(tmp_path: Path) -> None:
    with pytest.raises(InspectionError):
        load_inspectable(write_session(tmp_path / "s", outcome="insufficient_evidence"))


class _Bridge:
    def __init__(self, state: str, reason: str | None):
        status = {"request_id": "x", "state": state, "timestamp_ns": 7, "reason": reason}
        self.posts: list[dict] = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def _send(self, body):
                raw = json.dumps(body).encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_POST(self):  # noqa: N802
                outer.posts.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
                self._send({"status": status})

            def do_GET(self):  # noqa: N802
                self._send({"status": status, "trace": [status]})

            def log_message(self, *_):
                pass

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.httpd.server_port}"


def test_rejected_request_is_recorded_and_exits_nonzero(tmp_path: Path) -> None:
    session = write_session(tmp_path / "s")
    bridge = _Bridge("rejected", "stale or missing vehicle pose")
    out = tmp_path / "out"
    assert main(["inspect", str(session), "--bridge", bridge.url, "--output", str(out)]) == 1
    bridge.httpd.shutdown()
    assert len(bridge.posts) == 1
    lines = [json.loads(line) for line in (out / "inspection.jsonl").read_text().splitlines()]
    assert lines[-1]["event"] == "result" and lines[-1]["outcome"] == "rejected"
    assert not any(line["event"] == "observation_updated" for line in lines)


def test_observation_without_pose_never_reaches_the_bridge(tmp_path: Path) -> None:
    session = write_session(tmp_path / "s", pose=None)
    bridge = _Bridge("accepted", None)
    code = main(["inspect", str(session), "--bridge", bridge.url, "--output", str(tmp_path / "out")])
    bridge.httpd.shutdown()
    assert code == 1 and bridge.posts == []


def test_review_accepts_live_observation_events_with_telemetry() -> None:
    from ringquad_harness.evaluate import observation_from_event
    obs = observation_from_event({
        "observation_id": "o1", "session_id": "s", "camera_id": "cam", "first_capture_ns": 1, "last_capture_ns": 2,
        "bbox": [1, 2, 3, 4], "evidence_frame_ids": ["f"], "evidence": [], "pose": POSE, "world_target": None})
    assert obs.observation_id == "o1" and not hasattr(obs, "pose")
