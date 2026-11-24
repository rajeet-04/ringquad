from __future__ import annotations

import time
from pathlib import Path
from typing import Iterator

import av

from .schemas import Frame


class VideoReplayError(ValueError):
    """The clip cannot provide a valid replay timeline."""


def iter_video(
    path: Path,
    session_id: str,
    camera_id: str,
    *,
    paced: bool = False,
    sample_every: int = 1,
) -> Iterator[Frame]:
    """Decode frames and preserve normalized presentation timestamps.

    Sequence numbers refer to decoded frames, so sampled frames retain gaps.
    """
    if sample_every < 1:
        raise ValueError("sample_every must be at least 1")
    try:
        container = av.open(str(path))
    except (av.error.FFmpegError, OSError) as exc:
        raise VideoReplayError(f"cannot open video {path}: {exc}") from exc

    with container:
        streams = container.streams.video
        if not streams:
            raise VideoReplayError(f"no video stream in {path}")
        stream = streams[0]
        first_pts_ns: int | None = None
        previous_pts_ns: int | None = None
        wall_start: float | None = None
        origin_ns: int | None = None
        decoded_index = 0
        try:
            for decoded_index, frame in enumerate(container.decode(stream)):
                if frame.pts is None or frame.time_base is None:
                    raise VideoReplayError(f"frame {decoded_index} has no presentation timestamp/time base")
                pts_ns = int(frame.pts * frame.time_base * 1_000_000_000)
                if previous_pts_ns is not None and pts_ns < previous_pts_ns:
                    raise VideoReplayError(f"frame {decoded_index} presentation time regressed")
                previous_pts_ns = pts_ns
                if first_pts_ns is None:
                    first_pts_ns = pts_ns
                    origin_ns = pts_ns
                    wall_start = time.monotonic()
                capture_ns = pts_ns - origin_ns
                if capture_ns < 0:
                    raise VideoReplayError(f"frame {decoded_index} precedes the first presentation time")
                if paced and wall_start is not None:
                    delay = capture_ns / 1_000_000_000 - (time.monotonic() - wall_start)
                    if delay > 0:
                        time.sleep(delay)
                if decoded_index % sample_every == 0:
                    yield Frame(
                        session_id=session_id,
                        camera_id=camera_id,
                        sequence=decoded_index,
                        capture_ns=capture_ns,
                        clock_domain="video_presentation",
                        image=frame.to_ndarray(format="rgb24"),
                    )
        except (av.error.FFmpegError, OSError) as exc:
            raise VideoReplayError(f"failed decoding {path}: {exc}") from exc
        if first_pts_ns is None:
            raise VideoReplayError(f"video contains no decoded frames: {path}")
