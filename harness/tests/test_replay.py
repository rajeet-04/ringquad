from __future__ import annotations

import json
from pathlib import Path

import av
import numpy as np
from fractions import Fraction

from ringquad_harness.cli import main
from ringquad_harness.video import iter_video


def make_video(path: Path) -> None:
    container = av.open(str(path), mode="w", format="mp4")
    stream = container.add_stream("mpeg4", rate=25)
    stream.width = 32
    stream.height = 24
    stream.pix_fmt = "yuv420p"
    stream.time_base = Fraction(1, 1000)
    for pts in (0, 40, 115):
        frame = av.VideoFrame.from_ndarray(np.full((24, 32, 3), pts % 255, dtype=np.uint8), format="rgb24")
        frame.pts = pts
        frame.time_base = Fraction(1, 1000)
        for packet in stream.encode(frame):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()


def test_decodes_presentation_times_in_order_and_sampling_keeps_gap(tmp_path: Path) -> None:
    clip = tmp_path / "timestamped.mp4"
    make_video(clip)

    frames = list(iter_video(clip, "s", "cam"))
    assert [f.capture_ns for f in frames] == [0, 40_000_000, 115_000_000]
    assert [f.sequence for f in frames] == [0, 1, 2]
    sampled = list(iter_video(clip, "s", "cam", sample_every=2))
    assert [(f.sequence, f.capture_ns) for f in sampled] == [(0, 0), (2, 115_000_000)]
    assert sampled[1].frame_id == "s:cam:2"


def test_replay_writes_events_and_completed_manifest(tmp_path: Path, monkeypatch) -> None:
    clip = tmp_path / "timestamped.mp4"
    out = tmp_path / "session"
    make_video(clip)
    monkeypatch.setattr("sys.argv", ["ringquad-harness", "replay", str(clip), "--output", str(out), "--sample-every", "2"])

    assert main() == 0
    events = [json.loads(line) for line in (out / "events.jsonl").read_text().splitlines()]
    manifest = json.loads((out / "manifest.json").read_text())
    assert [event["timestamp_ns"] for event in events] == [0, 115_000_000]
    assert [event["sequence"] for event in events] == [0, 2]
    assert manifest["statistics"] == {"decoded": 3, "processed": 2, "skipped": 1, "dropped": 0}


def test_invalid_video_returns_nonzero_without_completed_manifest(tmp_path: Path, monkeypatch) -> None:
    clip = tmp_path / "invalid.mp4"
    clip.write_bytes(b"not a video")
    out = tmp_path / "session"
    monkeypatch.setattr("sys.argv", ["ringquad-harness", "replay", str(clip), "--output", str(out)])

    assert main() != 0
    assert not (out / "manifest.json").exists()
