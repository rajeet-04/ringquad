from __future__ import annotations

import asyncio
import json
from pathlib import Path
import time
from typing import Any
import uuid

from .agent import ReviewBackend, ReviewRunner
from .schemas import Decision, EvidenceFrame, Observation, SessionEvent
from .session import SessionWriter
from .video import iter_video
from .pipeline import PerceptionPipeline


def observation_from_event(data: dict[str, Any]) -> Observation:
    data = dict(data)
    data["evidence_frame_ids"] = tuple(data.get("evidence_frame_ids", ()))
    data["evidence"] = tuple(EvidenceFrame(**item) for item in data.get("evidence", ()))
    return Observation(**data)


async def review_session(session_dir: Path, backend: ReviewBackend, *, timeout: float = 60,
                         queue_size: int = 1) -> dict[str, Any]:
    """Review observations in a recorded perception session and append auditable events."""
    session_dir = Path(session_dir)
    manifest = json.loads((session_dir / "manifest.json").read_text())
    if manifest.get("stage") == "review":
        raise ValueError(f"session already reviewed: {session_dir}")
    observations = []
    with (session_dir / manifest["events"]).open(encoding="utf-8") as source:
        for line in source:
            if not line:
                continue
            event = json.loads(line)
            if event["event_type"] != "observation":
                continue
            observation = observation_from_event(event["data"])
            def resolve(path: str) -> str:
                candidate = Path(path)
                return str(candidate if candidate.is_absolute() else (session_dir / candidate).resolve())
            evidence = tuple(EvidenceFrame(ev.frame_id, ev.capture_ns,
                                           resolve(ev.context_path), resolve(ev.crop_path))
                             for ev in observation.evidence)
            observations.append(Observation(**{**observation.__dict__, "evidence": evidence,
                                               "context_path": resolve(observation.context_path) if observation.context_path else None,
                                               "crop_path": resolve(observation.crop_path) if observation.crop_path else None}))
    runner = ReviewRunner(backend, timeout=timeout, queue_size=queue_size)
    worker = asyncio.create_task(runner.run())
    for observation in observations:
        await runner.submit(observation)
    await runner.close()
    results = await worker
    now = time.time_ns()
    with (session_dir / manifest["events"]).open("a", encoding="utf-8") as events:
        for observation, result in results:
            data: dict[str, Any] = {"observation_id": observation.observation_id,
                                    "status": "failed" if result.error else "completed",
                                    "proposal_count": result.proposals,
                                    "error": result.error,
                                    "clock_domain": "host_wall",
                                    "observation_capture_ns": observation.last_capture_ns,
                                    "model_metrics": getattr(backend, "last_metrics", {})}
            if result.decision:
                data["decision"] = result.decision.model_dump(mode="json")
            events.write(SessionEvent(event_type="review", timestamp_ns=now,
                                      session_id=observation.session_id, camera_id=observation.camera_id,
                                      data=data).model_dump_json() + "\n")
    summary = {"observations": len(observations), "reviews": len(results),
               "inspection_proposals": sum(r.proposals for _, r in results),
               "failures": sum(r.error is not None for _, r in results)}
    manifest.setdefault("statistics", {}).update({"reviewed": summary["reviews"],
        "inspection_proposals": summary["inspection_proposals"], "review_failures": summary["failures"]})
    manifest["stage"] = "review"
    (session_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return summary


def evaluate_manifest(manifest_path: Path, output_dir: Path, media_dir: Path,
                      backend: ReviewBackend, *, timeout: float = 60) -> tuple[dict[str, Any], int]:
    """Replay each manifest clip deterministically and compare review outcome to expectation."""
    from .cli import replay
    manifest_path, output_dir, media_dir = map(Path, (manifest_path, output_dir, media_dir))
    data = json.loads(manifest_path.read_text())
    output_dir.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {"manifest": str(manifest_path), "model": str(getattr(backend, "model_path", "custom")),
                              "cases": []}
    mismatches = 0
    for clip in data["clips"]:
        clip_dir = output_dir / clip["clip_id"]
        code = replay(media_dir / Path(clip["path"]).name, clip_dir, "perception")
        if code:
            mismatches += 1
            report["cases"].append({"clip_id": clip["clip_id"], "error": "perception replay failed", "return_code": code})
            continue
        started = time.monotonic()
        summary = asyncio.run(review_session(clip_dir, backend, timeout=timeout))
        rows = [json.loads(line) for line in (clip_dir / "events.jsonl").read_text().splitlines() if line]
        reviews = [row["data"] for row in rows if row["event_type"] == "review"]
        outcomes = [r["decision"]["outcome"] for r in reviews if r.get("decision")]
        expected = clip.get("expected_review_outcome")
        expected_normalized = {"inspect_candidate": "inspect", "monitor": "monitor"}.get(expected, expected)
        proposal_ok = summary["inspection_proposals"] == (1 if expected_normalized == "inspect" else 0)
        outcome_ok = bool(outcomes) and expected_normalized in outcomes
        if expected_normalized == "monitor" and not outcomes:
            outcome_ok = summary["observations"] == 0
        passed = proposal_ok and outcome_ok and summary["failures"] == 0
        mismatches += not passed
        report["cases"].append({"clip_id": clip["clip_id"], "expected": expected_normalized,
                                "outcomes": outcomes, "summary": summary, "passed": passed,
                                "review_metrics": getattr(backend, "last_metrics", {}),
                                "elapsed_seconds": round(time.monotonic() - started, 3),
                                "evidence": [ev["data"].get("evidence", []) for ev in rows if ev["event_type"] == "observation"]})
    report["return_code"] = 1 if mismatches else 0
    (output_dir / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report, report["return_code"]
