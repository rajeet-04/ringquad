"""Close one inspection loop: reviewed observation -> bridge request -> closer view -> linked evidence.

Reads a reviewed live session (events.jsonl), submits an inspection request to the bridge,
polls until a terminal status, then collects a fresh, lower-altitude frame from the live
stream that still shows the anomaly and links it to the original observation ID.
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

import cv2

from .live import LiveIngest, LiveStreamError, iter_records
from .perception import detect_candidates

TERMINAL = {"completed", "failed", "rejected"}


class InspectionError(RuntimeError):
    pass


def load_inspectable(session: Path) -> dict:
    """First observation whose completed review says `inspect`, with the context needed for a request."""
    events = [json.loads(line) for line in (session / "events.jsonl").read_text().splitlines() if line]
    calibrations = {e["data"]["calibration_id"]: e["data"]["calibration"] for e in events if e["event_type"] == "calibration"}
    reviews = {e["data"]["observation_id"]: e["data"] for e in events if e["event_type"] == "review"}
    for event in events:
        if event["event_type"] != "observation":
            continue
        data = event["data"]
        review = reviews.get(data["observation_id"])
        decision = (review or {}).get("decision")
        if review and review.get("status") == "completed" and decision and decision["outcome"] == "inspect":
            frame = next(e for e in events if e["event_type"] == "frame" and e["frame_id"] == event["frame_id"])
            cal_id = frame["data"].get("calibration_id")
            return {"observation": data, "decision": decision, "capture_ns": event["timestamp_ns"],
                    "pose": data.get("pose"), "calibration": calibrations.get(cal_id), "frame_data": frame["data"]}
    raise InspectionError("session has no reviewed observation with an inspect decision")


def build_request(item: dict) -> dict:
    observation, decision = item["observation"], item["decision"]
    return {
        "request_id": str(uuid.uuid4()),
        "observation_id": observation["observation_id"],
        "evidence_ids": list(decision["evidence_frame_ids"]),
        "requested_view": decision["requested_view"],
        "bbox": list(observation["bbox"]),
        "capture_ns": item["capture_ns"],
        "pose": item["pose"],
        "calibration": item["calibration"],
    }


class BridgeClient:
    def __init__(self, base_url: str, timeout: float = 10.0):
        self.base, self.timeout = base_url.rstrip("/"), timeout

    def _call(self, request) -> dict:
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as resp:
                return json.loads(resp.read())
        except urllib.error.HTTPError as exc:
            raise InspectionError(f"bridge returned HTTP {exc.code}: {exc.read()[:200]!r}") from exc
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise InspectionError(f"bridge unreachable: {exc}") from exc

    def submit(self, request: dict) -> dict:
        req = urllib.request.Request(f"{self.base}/inspection-requests", data=json.dumps(request).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        return self._call(req)["status"]

    def get(self, request_id: str) -> dict:
        return self._call(urllib.request.Request(f"{self.base}/inspection-requests/{request_id}"))


def _write(out, kind: str, **data) -> None:
    out.write(json.dumps({"event": kind, "wall_time": time.time(), **data}) + "\n")
    out.flush()


def collect_followup(stream_url: str, item: dict, completed_ns: int, output: Path, *, timeout: float,
                     min_descent_m: float = 0.15) -> dict | None:
    """First frame after completion, clearly lower than the original view, that still shows a candidate."""
    start_z = item["pose"]["position"][2] if item["pose"] else None
    ingest = LiveIngest("followup", max_pose_age_ns=2_000_000_000)
    deadline = time.monotonic() + timeout
    stream = iter_records(lambda: urllib.request.urlopen(stream_url, timeout=10), reconnect_delay=0.5, max_reconnects=5)
    for kind, payload in stream:
        if time.monotonic() > deadline:
            return None
        if kind != "record":
            continue
        try:
            result = ingest.handle(*payload)
        except LiveStreamError:
            continue
        if result is None or result.frame.capture_ns <= completed_ns or result.pose is None:
            continue
        if start_z is not None and result.pose.position[2] > start_z - min_descent_m:
            continue
        candidates = detect_candidates(result.frame)
        if not candidates:
            continue
        output.mkdir(parents=True, exist_ok=True)
        path = output / f"followup-{result.frame.sequence:06d}.jpg"
        cv2.imwrite(str(path), result.frame.image[:, :, ::-1])
        best = max(candidates, key=lambda c: c.score)
        return {"frame_id": result.frame.frame_id, "capture_ns": result.frame.capture_ns,
                "pose": result.pose.to_dict(), "bbox": list(best.bbox), "image_path": path.name}
    return None


def run_inspection(session: Path, bridge_url: str, output: Path, *, poll_timeout: float = 90.0,
                   followup_timeout: float = 30.0) -> int:
    session, output = Path(session), Path(output)
    try:
        item = load_inspectable(session)
        if not item["pose"] or not item["calibration"]:
            raise InspectionError("observation has no pose or calibration: image-space only, no robot request")
    except (InspectionError, OSError, KeyError, StopIteration, json.JSONDecodeError) as exc:
        print(f"inspect failed: {exc}")
        return 1
    output.mkdir(parents=True, exist_ok=False)
    client = BridgeClient(bridge_url)
    request = build_request(item)
    with (output / "inspection.jsonl").open("x") as out:
        try:
            _write(out, "request", **{k: v for k, v in request.items() if k != "calibration"})
            status = client.submit(request)
            deadline = time.monotonic() + poll_timeout
            while status["state"] not in TERMINAL:
                if time.monotonic() > deadline:
                    raise InspectionError("timed out waiting for a terminal inspection status")
                time.sleep(0.5)
                status = client.get(request["request_id"])["status"]
            trace = client.get(request["request_id"])["trace"]
            _write(out, "trace", request_id=request["request_id"], trace=trace)
            if status["state"] != "completed":
                _write(out, "result", outcome=status["state"], reason=status.get("reason"))
                print(f"inspection {status['state']}: {status.get('reason')}")
                return 1
            followup = collect_followup(bridge_url.rstrip("/") + "/stream", item, status["timestamp_ns"],
                                        output / "evidence", timeout=followup_timeout)
            if followup is None:
                _write(out, "result", outcome="no_followup", reason="no closer frame showing the anomaly arrived")
                print("inspection completed but no closer view arrived")
                return 1
            _write(out, "observation_updated", observation_id=request["observation_id"],
                   request_id=request["request_id"], followup=followup)
            _write(out, "result", outcome="closed", observation_id=request["observation_id"])
        except InspectionError as exc:
            _write(out, "result", outcome="error", reason=str(exc))
            print(f"inspect failed: {exc}")
            return 1
    print(json.dumps({"output": str(output), "request_id": request["request_id"]}))
    return 0
