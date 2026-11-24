from __future__ import annotations

import asyncio
import json
from pathlib import Path
import time
from typing import Any
import uuid

from .agent import ReviewBackend, ReviewRunner
from .schemas import Decision, EvidenceFrame, Observation, SessionEvent
from dataclasses import fields

from .session import SessionWriter
from .video import iter_video
from .pipeline import PerceptionPipeline


def observation_from_event(data: dict[str, Any]) -> Observation:
    data = dict(data)
    data["evidence_frame_ids"] = tuple(data.get("evidence_frame_ids", ()))
    data["evidence"] = tuple(EvidenceFrame(**item) for item in data.get("evidence", ()))
    # Live sessions add telemetry (pose, world_target) the review does not use.
    known = {f.name for f in fields(Observation)}
    return Observation(**{k: v for k, v in data.items() if k in known})


async def review_session(session_dir: Path, backend: ReviewBackend, *, timeout: float = 60,
                         queue_size: int = 1) -> dict[str, Any]:
    """Stream a recorded session through one bounded reviewer and append auditable results."""
    session_dir = Path(session_dir)
    root = session_dir.resolve()
    manifest = json.loads((session_dir / "manifest.json").read_text())
    if manifest.get("stage") == "review":
        raise ValueError(f"session already reviewed: {session_dir}")
    events_path = Path(manifest["events"])
    events_path = (events_path if events_path.is_absolute() else root / events_path).resolve()
    if not events_path.is_relative_to(root):
        raise ValueError("event log path escapes session bundle")
    run_id = str(uuid.uuid4())

    def resolve_bundle_path(path: str) -> tuple[str, str]:
        candidate = Path(path)
        resolved = (candidate if candidate.is_absolute() else root / candidate).resolve()
        if not resolved.is_relative_to(root):
            raise ValueError(f"evidence path escapes session bundle: {path}")
        if not resolved.is_file():
            raise ValueError(f"evidence file is missing from session bundle: {path}")
        return str(resolved), resolved.relative_to(root).as_posix()

    with events_path.open("a", encoding="utf-8") as event_out:
        def write_result(observation: Observation, result) -> None:
            evidence_by_id = {item.frame_id: item for item in observation.evidence}
            selected_ids = result.decision.evidence_frame_ids if result.decision else ()
            if any(frame_id not in evidence_by_id for frame_id in selected_ids):
                raise ValueError("decision evidence frame has no corresponding bundle file references")
            refs = []
            for frame_id in selected_ids:
                item = evidence_by_id[frame_id]
                _, context_rel = resolve_bundle_path(item.context_path)
                _, crop_rel = resolve_bundle_path(item.crop_path)
                refs.append({"frame_id": item.frame_id, "capture_ns": item.capture_ns,
                             "context_path": context_rel, "crop_path": crop_rel})
            data: dict[str, Any] = {"observation_id": observation.observation_id,
                                    "status": "failed" if result.error else "completed",
                                    "proposal_count": result.proposals,
                                    "error": result.error,
                                    "clock_domain": "host_wall",
                                    "observation_capture_ns": observation.last_capture_ns,
                                    "model_metrics": result.metrics or {},
                                    "evidence_refs": refs,
                                    "run_id": run_id,
                                    "model": getattr(backend, "metadata", {"provider": "custom"})}
            if result.decision:
                data["decision"] = result.decision.model_dump(mode="json")
            event_out.write(SessionEvent(event_type="review", timestamp_ns=time.time_ns(),
                                         session_id=observation.session_id, camera_id=observation.camera_id,
                                         data=data).model_dump_json() + "\n")
            event_out.flush()

        runner = ReviewRunner(backend, timeout=timeout, queue_size=queue_size,
                              on_result=write_result, collect_results=False)
        # Freeze the source boundary before appending review records to this same file.
        source_limit = events_path.stat().st_size

        async def produce() -> None:
            with events_path.open(encoding="utf-8") as source:
                while source.tell() < source_limit:
                    line = source.readline()
                    if not line:
                        break
                    event = json.loads(line)
                    if event["event_type"] != "observation":
                        continue
                    observation = observation_from_event(event["data"])
                    fixed_evidence = []
                    for ev in observation.evidence:
                        context, _ = resolve_bundle_path(ev.context_path)
                        crop, _ = resolve_bundle_path(ev.crop_path)
                        fixed_evidence.append(EvidenceFrame(ev.frame_id, ev.capture_ns, context, crop))
                    observation = Observation(**{**observation.__dict__, "evidence": tuple(fixed_evidence),
                        "context_path": resolve_bundle_path(observation.context_path)[0] if observation.context_path else None,
                        "crop_path": resolve_bundle_path(observation.crop_path)[0] if observation.crop_path else None})
                    await runner.submit(observation)
            await runner.close()

        producer = asyncio.create_task(produce())
        worker = asyncio.create_task(runner.run())
        try:
            done, pending = await asyncio.wait((producer, worker), return_when=asyncio.FIRST_EXCEPTION)
            for task in done:
                task.result()
            if pending:
                await asyncio.gather(*pending)
        except BaseException:
            for task in (producer, worker):
                if not task.done():
                    task.cancel()
            await asyncio.gather(producer, worker, return_exceptions=True)
            raise
    summary = {"observations": len(runner._seen), "reviews": runner.completed,
               "inspection_proposals": runner.proposals, "failures": runner.failures,
               "model": getattr(backend, "metadata", {"provider": "custom"}), "run_id": run_id}
    manifest.setdefault("statistics", {}).update({"reviewed": summary["reviews"],
        "inspection_proposals": summary["inspection_proposals"], "review_failures": summary["failures"]})
    manifest["stage"] = "review"
    manifest["review"] = {"run_id": run_id, "model": summary["model"]}
    (session_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return summary


def evaluate_manifest(manifest_path: Path, output_dir: Path, media_dir: Path,
                      backend: ReviewBackend, *, timeout: float = 60) -> tuple[dict[str, Any], int]:
    """Replay each manifest clip deterministically and compare review outcome to expectation."""
    from .cli import replay
    manifest_path, output_dir, media_dir = map(Path, (manifest_path, output_dir, media_dir))
    data = json.loads(manifest_path.read_text())
    clips = data.get("clips")
    if not isinstance(clips, list) or not clips:
        raise ValueError("fixture manifest must contain a non-empty clips list")
    recognized = {"inspect_candidate": "inspect", "inspect": "inspect",
                  "monitor": "monitor", "insufficient_evidence": "insufficient_evidence"}
    for clip in clips:
        clip_id = clip.get("clip_id", "<missing>")
        expected_count = clip.get("expected_observations")
        if not isinstance(expected_count, int) or isinstance(expected_count, bool) or expected_count < 0:
            raise ValueError(f"fixture {clip_id} requires non-negative integer expected_observations")
        expected_raw = clip.get("expected_review_outcome")
        if expected_raw not in recognized:
            raise ValueError(f"fixture {clip_id} has missing or unknown expected_review_outcome: {expected_raw!r}")
        if expected_count == 0 and recognized[expected_raw] != "monitor":
            raise ValueError(f"zero-observation fixture {clip_id} must expect monitor (no model review)")
        if "clip_id" not in clip or "path" not in clip:
            raise ValueError("each fixture requires clip_id and path")
        clip_id_value = clip["clip_id"]
        if not isinstance(clip_id_value, str) or not clip_id_value or Path(clip_id_value).name != clip_id_value:
            raise ValueError(f"fixture clip_id must be a safe path component: {clip_id_value!r}")
        media_rel = Path(clip["path"])
        if media_rel.is_absolute() or ".." in media_rel.parts:
            raise ValueError(f"fixture {clip_id_value} media path must be relative and remain in media-dir")
    clip_ids = [clip["clip_id"] for clip in clips]
    if len(set(clip_ids)) != len(clip_ids):
        raise ValueError("fixture clip_id values must be unique")
    output_dir.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {"manifest": str(manifest_path), "model": getattr(backend, "metadata", {"provider": "custom"}),
                              "run_id": str(uuid.uuid4()), "cases": []}
    mismatches = 0
    for clip in data["clips"]:
        clip_dir = output_dir / clip["clip_id"]
        code = replay(media_dir / Path(clip["path"]), clip_dir, "perception")
        if code:
            mismatches += 1
            report["cases"].append({"clip_id": clip["clip_id"], "error": "perception replay failed", "return_code": code})
            continue
        started = time.monotonic()
        summary = asyncio.run(review_session(clip_dir, backend, timeout=timeout))
        reviews = []
        observation_evidence = []
        first_capture = last_capture = None
        frame_count = 0
        with (clip_dir / "events.jsonl").open(encoding="utf-8") as rows:
            for line in rows:
                row = json.loads(line)
                if row["event_type"] == "review":
                    reviews.append(row["data"])
                elif row["event_type"] == "observation":
                    evs = row["data"].get("evidence", [])
                    observation_evidence.extend(evs)
                elif row["event_type"] == "frame":
                    ts = row["timestamp_ns"]
                    first_capture = ts if first_capture is None else min(first_capture, ts)
                    last_capture = ts if last_capture is None else max(last_capture, ts)
                    frame_count += 1
        outcomes = [r["decision"]["outcome"] for r in reviews if r.get("decision")]
        expected_count = clip["expected_observations"]
        expected_normalized = recognized[clip["expected_review_outcome"]]
        count_ok = summary["observations"] == expected_count
        proposal_ok = summary["inspection_proposals"] == (expected_count if expected_normalized == "inspect" else 0)
        outcome_ok = (not outcomes) if expected_count == 0 else (len(outcomes) == expected_count and all(o == expected_normalized for o in outcomes))
        passed = count_ok and proposal_ok and outcome_ok and summary["failures"] == 0
        mismatches += not passed
        report["cases"].append({"clip_id": clip["clip_id"], "expected": expected_normalized,
                                "expected_observations": expected_count, "observed_observations": summary["observations"],
                                "outcomes": outcomes, "summary": summary, "passed": passed,
                                "review_metrics": [r.get("model_metrics", {}) for r in reviews],
                                "elapsed_seconds": round(time.monotonic() - started, 3),
                                "frame_statistics": json.loads((clip_dir / "manifest.json").read_text()).get("statistics", {}),
                                "capture_timing": {"clock_domain": "video_pts_relative", "frames": frame_count,
                                                   "first_capture_ns": first_capture, "last_capture_ns": last_capture,
                                                   "duration_ns": (last_capture - first_capture) if first_capture is not None else 0},
                                "evidence": observation_evidence})
    report["return_code"] = 1 if mismatches else 0
    (output_dir / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report, report["return_code"]
