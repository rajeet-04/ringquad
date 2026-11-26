#!/usr/bin/env python3
"""Render a real clip through the harness into one annotated mp4 (offline run, recorded).

Pass 1 runs the detector and tracker over a time window and reviews the first confirmed tracks with the local
Qwen3-VL model (failed reviews are retried). Pass 2 re-plays the window with boxes, outcomes and explanations.
Outcomes are whatever the model returned; nothing is filtered by decision. The video says it is a recorded run.

usage: render_real_demo.py CLIP.mp4 OUT.mp4 --model MODEL_DIR [--start 0] [--end 20] [--max-reviews 8]
       [--credit "Source text"] [--json OUT.json]
Run through the harness env: uv run --project harness python harness/eval/render_real_demo.py ...
"""
import argparse
import asyncio
import json
import subprocess
import tempfile
import textwrap
import uuid
from pathlib import Path

import cv2
import numpy as np

from ringquad_harness.agent import validate_decision
from ringquad_harness.model_mlx import MLXReviewBackend
from ringquad_harness.perception import detect_candidates
from ringquad_harness.pipeline import PerceptionPipeline
from ringquad_harness.video import iter_video

COLORS = {"inspect": (60, 60, 255), "monitor": (0, 200, 255), "insufficient_evidence": (180, 180, 180),
          "failed": (255, 0, 255), "pending": (0, 255, 0)}
SAMPLE = 3
W, PANEL = 960, 480


def frames(path: Path, start: float, end: float):
    for f in iter_video(path, str(uuid.uuid4()), "demo", sample_every=SAMPLE):
        t = f.capture_ns / 1e9
        if t < start:
            continue
        if t > end:
            return
        yield t, f


def review(backend, obs, retries: int):
    async def go():
        d = validate_decision(await backend.review(obs, obs.evidence), obs)
        return d.outcome, d.explanation
    err = ""
    for attempt in range(1, retries + 2):
        try:
            outcome, text = asyncio.run(go())
            return outcome, text, attempt
        except Exception as exc:
            err = f"{type(exc).__name__}: {exc}"[:120]
    return "failed", err, retries + 1


def put(img, lines, x, y, scale=0.5, color=(235, 235, 235), step=20):
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += step
    return y


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("clip", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=20.0)
    ap.add_argument("--max-reviews", type=int, default=8)
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--credit", default="")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()

    backend = MLXReviewBackend(a.model, timeout=120.0)
    obs_list, cands, shape = [], {}, None
    with tempfile.TemporaryDirectory() as tmp:
        pipe = PerceptionPipeline(Path(tmp))
        for t, f in frames(a.clip, a.start, a.end):
            shape = f.image.shape
            cs = detect_candidates(f, pipe.perception)
            cands[f.sequence] = [c.bbox for c in cs]
            for o in pipe.tracker.update(f, cs):
                obs_list.append({"obs": o, "t": t, "seq": f.sequence})
        print(f"{len(obs_list)} tracks in window; reviewing the first {a.max_reviews}", flush=True)
        results = []
        for item in obs_list[: a.max_reviews]:
            outcome, text, attempts = review(backend, item["obs"], a.retries)
            item.update(outcome=outcome, text=text, attempts=attempts)
            results.append({"t": round(item["t"], 1), "outcome": outcome, "attempts": attempts, "explanation": text})
            print(f"t={item['t']:.1f}s {outcome} (attempts {attempts}): {text[:100]}", flush=True)
        for item in obs_list[a.max_reviews:]:
            item["outcome"] = "skip"
        if a.json:
            a.json.write_text(json.dumps(results, indent=2))

        H = int(shape[0] * W / shape[1])
        ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                               "-s", f"{W + PANEL}x{H}", "-r", "10", "-i", "-", "-c:v", "libx264", "-crf", "26",
                               "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(a.out)],
                              stdin=subprocess.PIPE)
        scale = W / shape[1]
        n = 0
        for t, f in frames(a.clip, a.start, a.end):
            img = cv2.resize(cv2.cvtColor(f.image, cv2.COLOR_RGB2BGR), (W, H))
            for x0, y0, x1, y1 in cands.get(f.sequence, []):
                cv2.rectangle(img, (int(x0 * scale), int(y0 * scale)), (int(x1 * scale), int(y1 * scale)), (0, 200, 255), 1)
            shown = [i for i in obs_list if i["outcome"] != "skip" and t >= i["t"] + 1.0]
            for i in obs_list:
                if i["outcome"] == "skip" or t < i["t"] or t > i["t"] + 6.0:
                    continue
                col = COLORS[i["outcome"]] if i in shown else COLORS["pending"]
                x0, y0, x1, y1 = (int(v * scale) for v in i["obs"].bbox)
                cv2.rectangle(img, (x0, y0), (x1, y1), col, 3)
            side = np.full((H, PANEL, 3), 24, np.uint8)
            y = put(side, ["HARNESS ON REAL FOOTAGE"], 14, 30, 0.7, (0, 220, 255), 30)
            counts = {k: sum(1 for i in shown if i["outcome"] == k) for k in COLORS}
            y = put(side, [f"clip time {t:5.1f} s    tracks {sum(1 for i in obs_list if i['t'] <= t)}",
                           f"reviewed {len(shown)}   inspect {counts['inspect']}   monitor {counts['monitor']}"], 14, y + 4)
            y += 8
            put(side, ["orange: detector candidates", "green: reviewing   red: inspect   yellow: monitor",
                       "grey: insufficient evidence   magenta: review failed"], 14, y, 0.4, (160, 160, 160), 17)
            y += 66
            last = shown[-1] if shown else None
            if last:
                put(side, [f"decision: {last['outcome']}"], 14, y, 0.7, tuple(int(c) for c in COLORS[last["outcome"]]), 28)
                y += 32
                put(side, (lambda w: w[:14] if len(w) <= 14 else w[:13] + [w[13][:46] + " ..."])(textwrap.wrap(last["text"], 52)), 14, y, 0.42, (210, 210, 210), 17)
            else:
                put(side, ["waiting for a confirmed track..."], 14, y, 0.55, (0, 220, 255))
            put(side, ["Recorded offline run. Reviews by local Qwen3-VL-2B;", "failed reviews retried. Not a live result."]
                + ([a.credit] if a.credit else []), 14, H - 54, 0.4, (130, 130, 130), 16)
            ff.stdin.write(np.hstack([img, side]).tobytes())
            n += 1
        ff.stdin.close()
        ff.wait()
    print(a.out, n, "frames")


if __name__ == "__main__":
    main()
