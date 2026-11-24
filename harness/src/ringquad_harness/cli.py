from __future__ import annotations

import argparse
import json
import uuid
from pathlib import Path

from .schemas import SessionEvent
from .session import SessionWriter
from .video import VideoReplayError, iter_video


def replay(video: Path, output: Path, stage: str, *, paced: bool = False, sample_every: int = 1) -> int:
    if sample_every < 1:
        print("replay failed: --sample-every must be at least 1")
        return 2
    session_id = str(uuid.uuid4())
    writer: SessionWriter | None = None
    try:
        writer = SessionWriter(output, config={"paced": paced, "sample_every": sample_every})
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
        writer.finalize(session_id=session_id, video=str(video), stage=stage)
    except (VideoReplayError, OSError, ValueError) as exc:
        if writer is not None:
            writer.close()
        print(f"replay failed: {exc}")
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
    replay_parser.add_argument("--stage", default="ingest")
    replay_parser.add_argument("--paced", action="store_true")
    replay_parser.add_argument("--sample-every", type=positive_int, default=1)
    args = parser.parse_args()
    if args.command == "replay":
        return replay(args.video, args.output, args.stage, paced=args.paced, sample_every=args.sample_every)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
