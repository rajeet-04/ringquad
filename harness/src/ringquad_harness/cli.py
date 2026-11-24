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
                    payload = asdict(observation)
                    for evidence in payload["evidence"]:
                        evidence["context_path"] = str(Path(evidence["context_path"]).relative_to(output))
                        evidence["crop_path"] = str(Path(evidence["crop_path"]).relative_to(output))
                    payload["context_path"] = str(Path(payload["context_path"]).relative_to(output)) if payload["context_path"] else None
                    payload["crop_path"] = str(Path(payload["crop_path"]).relative_to(output)) if payload["crop_path"] else None
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


def main() -> int:
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
    evaluate_parser = commands.add_parser("evaluate")
    evaluate_parser.add_argument("manifest", type=Path)
    evaluate_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser.add_argument("--media-dir", type=Path, default=Path("../photo-clips"))
    evaluate_parser.add_argument("--model-path", type=Path, required=True)
    evaluate_parser.add_argument("--review-timeout", type=float, default=60.0)
    args = parser.parse_args()
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
