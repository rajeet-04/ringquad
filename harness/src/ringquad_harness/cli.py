from __future__ import annotations

import argparse
import asyncio
from dataclasses import asdict
import json
import math
import uuid
from pathlib import Path

from .pipeline import PerceptionPipeline
from .schemas import SessionEvent
from .session import SessionWriter
from .video import VideoReplayError, iter_video


def observation_payload(observation, output: Path) -> dict:
    payload = asdict(observation)
    for evidence in payload["evidence"]:
        evidence["context_path"] = str(Path(evidence["context_path"]).relative_to(output))
        evidence["crop_path"] = str(Path(evidence["crop_path"]).relative_to(output))
    payload["context_path"] = str(Path(payload["context_path"]).relative_to(output)) if payload["context_path"] else None
    payload["crop_path"] = str(Path(payload["crop_path"]).relative_to(output)) if payload["crop_path"] else None
    return payload


def live(url: str, output: Path, *, max_frames: int | None, max_pose_age_ms: float,
         reconnect_delay: float, max_reconnects: int | None, stop_after_observations: int | None = None) -> int:
    import urllib.request
    from .live import LiveIngest, LiveStreamError, iter_records
    if max_frames is not None and max_frames < 1:
        print("live failed: --max-frames must be at least 1")
        return 2
    if not math.isfinite(max_pose_age_ms) or max_pose_age_ms < 0:
        print("live failed: --max-pose-age-ms must be a non-negative finite number")
        return 2
    session_id = str(uuid.uuid4())
    ingest = LiveIngest(session_id, max_pose_age_ns=int(max_pose_age_ms * 1e6))
    writer: SessionWriter | None = None
    try:
        writer = SessionWriter(output, config={"url": url, "max_pose_age_ms": max_pose_age_ms, "stage": "perception",
                                               "source": "live"})
        writer.stats.update({"stale_pose": 0, "reconnects": 0, "invalid": 0, "clock_resets": 0, "observations": 0})
        pipeline = PerceptionPipeline(output / "evidence")
        last_ns = 0
        last_status: dict = {}
        seen_calibrations: set = set()
        stream = iter_records(lambda: urllib.request.urlopen(url, timeout=10),
                              reconnect_delay=reconnect_delay, max_reconnects=max_reconnects)
        for kind, item in stream:
            if kind == "reconnect":
                writer.stats["reconnects"] += 1
                writer.write_event(SessionEvent(event_type="reconnect", timestamp_ns=last_ns,
                                                session_id=session_id, data={"reason": item}))
                continue
            meta, payload = item
            if meta.get("kind") == "status":
                status = {k: v for k, v in meta.items() if k != "kind"}
                # The controller republishes its state at 10 Hz; record only changes (z is noise-level churn).
                key = {k: (v.split(" z=")[0] if k == "state" and isinstance(v, str) else v) for k, v in status.items()}
                if any(last_status.get(k) != v for k, v in key.items()):
                    last_status.update(key)
                    writer.write_event(SessionEvent(event_type="status", timestamp_ns=last_ns,
                                                    session_id=session_id, data=status))
                continue
            try:
                result = ingest.handle(meta, payload)
            except LiveStreamError as exc:
                writer.stats["invalid"] += 1
                writer.write_event(SessionEvent(event_type="invalid_record", timestamp_ns=last_ns,
                                                session_id=session_id, data={"reason": str(exc)}))
                continue
            if result is None:
                continue
            frame = result.frame
            last_ns = frame.capture_ns
            writer.stats["decoded"] += 1
            writer.stats["processed"] += 1
            writer.stats["dropped"] = max(writer.stats["dropped"], int(meta.get("dropped_total", 0)))
            if result.clock_reset:
                writer.stats["clock_resets"] += 1
                pipeline = PerceptionPipeline(output / "evidence")
                writer.write_event(SessionEvent(event_type="clock_reset", timestamp_ns=frame.capture_ns,
                                                session_id=session_id, camera_id=frame.camera_id))
            cal_id = frame.calibration_id
            if cal_id and cal_id not in seen_calibrations and result.meta.get("calibration"):
                seen_calibrations.add(cal_id)
                writer.write_event(SessionEvent(event_type="calibration", timestamp_ns=frame.capture_ns,
                                                session_id=session_id, camera_id=frame.camera_id,
                                                data={"calibration_id": cal_id, "calibration": result.meta["calibration"]}))
            if result.pose is None:
                writer.stats["stale_pose"] += 1
            writer.write_event(SessionEvent(
                event_type="frame", timestamp_ns=frame.capture_ns, session_id=session_id,
                camera_id=frame.camera_id, sequence=frame.sequence, frame_id=frame.frame_id,
                data={"image_shape": list(frame.image.shape), "clock_domain": frame.clock_domain,
                      "calibration_id": frame.calibration_id,
                      "pose": result.pose.to_dict() if result.pose else None}))
            for observation in pipeline.update(frame):
                data = observation_payload(observation, output)
                # Image-space only: a robot-bound target needs a fresh pose and a resolved ground point.
                data.update(pose=result.pose.to_dict() if result.pose else None, world_target=None)
                writer.write_event(SessionEvent(
                    event_type="observation", timestamp_ns=frame.capture_ns, session_id=session_id,
                    camera_id=frame.camera_id, sequence=frame.sequence, frame_id=frame.frame_id, data=data))
                writer.stats["observations"] += 1
            if max_frames is not None and writer.stats["processed"] >= max_frames:
                break
            if stop_after_observations is not None and writer.stats["observations"] >= stop_after_observations:
                break
        writer.finalize(session_id=session_id, video=url, stage="perception")
    except (LiveStreamError, OSError, ValueError) as exc:
        if writer is not None:
            writer.close()
        print(f"live failed: {exc}")
        return 1
    print(json.dumps({"session_id": session_id, "output": str(output)}))
    return 0


def replay(video: Path, output: Path, stage: str, *, paced: bool = False, sample_every: int = 1,
           model_path: Path | None = None, review_timeout: float = 60.0) -> int:
    if sample_every < 1:
        print("replay failed: --sample-every must be at least 1")
        return 2
    if stage not in {"ingest", "perception", "review"}:
        print("replay failed: --stage must be ingest, perception, or review")
        return 2
    review_backend = None
    if stage == "review":
        if not math.isfinite(review_timeout) or review_timeout <= 0:
            print("review failed: --review-timeout must be a positive finite number")
            return 2
        if model_path is None or not Path(model_path).expanduser().is_dir():
            print("review failed: --model-path must name an existing local model directory")
            return 2
        from .model_mlx import MLXReviewBackend
        try:
            review_backend = MLXReviewBackend(model_path, timeout=review_timeout)
        except (OSError, ValueError) as exc:
            print(f"review failed: {exc}")
            return 2
    session_id = str(uuid.uuid4())
    writer: SessionWriter | None = None
    try:
        writer = SessionWriter(output, config={"paced": paced, "sample_every": sample_every, "stage": stage})
        pipeline = PerceptionPipeline(output / "evidence") if stage in {"perception", "review"} else None
        for frame in iter_video(video, session_id, "camera-0", paced=paced):
            writer.stats["decoded"] += 1
            if frame.sequence % sample_every:
                writer.stats["skipped"] += 1
                continue
            writer.stats["processed"] += 1
            writer.write_event(SessionEvent(
                event_type="frame", timestamp_ns=frame.capture_ns, session_id=session_id,
                camera_id=frame.camera_id, sequence=frame.sequence, frame_id=frame.frame_id,
                data={"image_shape": list(frame.image.shape)},
            ))
            if pipeline is not None:
                observations = pipeline.update(frame)
                for observation in observations:
                    payload = observation_payload(observation, output)
                    writer.write_event(SessionEvent(
                        event_type="observation", timestamp_ns=frame.capture_ns, session_id=session_id,
                        camera_id=frame.camera_id, sequence=frame.sequence, frame_id=frame.frame_id,
                        data=payload,
                    ))
                    writer.stats["observations"] = writer.stats.get("observations", 0) + 1
        writer.finalize(session_id=session_id, video=str(video), stage="perception" if stage == "review" else stage)
    except (VideoReplayError, OSError, ValueError) as exc:
        if writer is not None:
            writer.close()
        print(f"replay failed: {exc}")
        return 1
    if stage == "review":
        from .evaluate import review_session
        try:
            summary = asyncio.run(review_session(output, review_backend, timeout=review_timeout))
        except (OSError, ValueError, RuntimeError) as exc:
            print(f"review failed: {exc}")
            return 1
        if summary["failures"]:
            print(json.dumps({"session_id": session_id, "output": str(output), "review": summary}))
            return 1
    print(json.dumps({"session_id": session_id, "output": str(output)}))
    return 0


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="ringquad-harness")
    commands = parser.add_subparsers(dest="command", required=True)
    replay_parser = commands.add_parser("replay")
    replay_parser.add_argument("video", type=Path)
    replay_parser.add_argument("--output", type=Path, required=True)
    replay_parser.add_argument("--stage", choices=("ingest", "perception", "review"), default="ingest")
    replay_parser.add_argument("--paced", action="store_true")
    replay_parser.add_argument("--sample-every", type=positive_int, default=1)
    replay_parser.add_argument("--model-path", type=Path)
    replay_parser.add_argument("--review-timeout", type=float, default=60.0)
    live_parser = commands.add_parser("live")
    live_parser.add_argument("--url", required=True)
    live_parser.add_argument("--output", type=Path, required=True)
    live_parser.add_argument("--max-frames", type=positive_int)
    live_parser.add_argument("--max-pose-age-ms", type=float, default=200.0)
    live_parser.add_argument("--reconnect-delay", type=float, default=1.0)
    live_parser.add_argument("--max-reconnects", type=int)
    live_parser.add_argument("--stop-after-observations", type=positive_int)
    review_parser = commands.add_parser("review")
    review_parser.add_argument("session", type=Path)
    review_parser.add_argument("--model-path", type=Path, required=True)
    review_parser.add_argument("--review-timeout", type=float, default=60.0)
    inspect_parser = commands.add_parser("inspect")
    inspect_parser.add_argument("session", type=Path)
    inspect_parser.add_argument("--bridge", required=True)
    inspect_parser.add_argument("--output", type=Path, required=True)
    inspect_parser.add_argument("--poll-timeout", type=float, default=90.0)
    inspect_parser.add_argument("--followup-timeout", type=float, default=30.0)
    evaluate_parser = commands.add_parser("evaluate")
    evaluate_parser.add_argument("manifest", type=Path)
    evaluate_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser.add_argument("--media-dir", type=Path, default=Path("../photo-clips"))
    evaluate_parser.add_argument("--model-path", type=Path, required=True)
    evaluate_parser.add_argument("--review-timeout", type=float, default=60.0)
    args = parser.parse_args(argv)
    if args.command == "live":
        return live(args.url, args.output, max_frames=args.max_frames, max_pose_age_ms=args.max_pose_age_ms,
                    reconnect_delay=args.reconnect_delay, max_reconnects=args.max_reconnects,
                    stop_after_observations=args.stop_after_observations)
    if args.command == "review":
        from .evaluate import review_session
        from .model_mlx import MLXReviewBackend
        try:
            if not math.isfinite(args.review_timeout) or args.review_timeout <= 0:
                raise ValueError("--review-timeout must be a positive finite number")
            summary = asyncio.run(review_session(args.session, MLXReviewBackend(args.model_path, timeout=args.review_timeout),
                                                 timeout=args.review_timeout))
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            print(f"review failed: {exc}")
            return 1
        print(json.dumps(summary))
        return 1 if summary["failures"] else 0
    if args.command == "inspect":
        from .inspect_loop import run_inspection
        return run_inspection(args.session, args.bridge, args.output, poll_timeout=args.poll_timeout,
                              followup_timeout=args.followup_timeout)
    if args.command == "replay":
        return replay(args.video, args.output, args.stage, paced=args.paced, sample_every=args.sample_every,
                      model_path=args.model_path, review_timeout=args.review_timeout)
    if args.command == "evaluate":
        from .evaluate import evaluate_manifest
        from .model_mlx import MLXReviewBackend
        try:
            if not math.isfinite(args.review_timeout) or args.review_timeout <= 0:
                raise ValueError("--review-timeout must be a positive finite number")
            report, code = evaluate_manifest(args.manifest, args.output, args.media_dir,
                MLXReviewBackend(args.model_path, timeout=args.review_timeout), timeout=args.review_timeout)
        except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            print(f"evaluation failed: {exc}")
            return 2
        print(json.dumps(report, indent=2))
        return code
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
