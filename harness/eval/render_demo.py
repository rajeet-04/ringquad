#!/usr/bin/env python3
"""Render the recorded sim stream with harness overlays into one mp4.

usage: render_demo.py RECORDING_DIR SESSION_DIR INSPECTION_DIR OUT.mp4
Left: camera frame with live detector boxes and the confirmed track. Right: stage, decision, request state.
"""
import json, subprocess, sys, textwrap
from pathlib import Path

import cv2
import numpy as np

from ringquad_harness.perception import detect_candidates
from ringquad_harness.schemas import Frame

rec, session, insp, out = (Path(a) for a in sys.argv[1:5])
metas = [json.loads(l) for l in (rec / "index.jsonl").read_text().splitlines()]
frames = [m for m in metas if m.get("kind") == "frame"]
status = [m for m in metas if m.get("kind") == "status"]
events = [json.loads(l) for l in (session / "events.jsonl").read_text().splitlines()]
obs = next(e["data"] for e in events if e["event_type"] == "observation")
rev = next(e["data"] for e in events if e["event_type"] == "review" and e["data"].get("decision"))
dec = rev["decision"]
log = [json.loads(l) for l in (insp / "inspection.jsonl").read_text().splitlines()]
trace = next(l["trace"] for l in log if l["event"] == "trace")
t_follow = next((l["followup"]["capture_ns"] for l in log if l["event"] == "observation_updated"), None)
t_obs = obs["last_capture_ns"]
t_start = min(e["timestamp_ns"] for e in events if e["event_type"] == "frame") - 3_000_000_000
frames = [m for m in frames if m["capture_ns"] >= t_start]
t_acc = trace[0]["timestamp_ns"]
states = [(t["timestamp_ns"], t["state"]) for t in trace]
W, H = 640, 480
panel_w = 400


def state_at(t):
    cur = None
    for ts, s in states:
        if ts <= t:
            cur = s
    return cur


def put(img, lines, x, y, scale=0.55, color=(235, 235, 235), step=22):
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += step
    return y


ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W + panel_w}x{H}",
                       "-r", "10", "-i", "-", "-c:v", "libx264", "-crf", "30", "-preset", "slow", "-pix_fmt", "yuv420p",
                       "-movflags", "+faststart", str(out)], stdin=subprocess.PIPE)
for m in frames:
    t = m["capture_ns"]
    img = cv2.imread(str(rec / m["file"]))
    rgb = img[:, :, ::-1].copy()
    for c in detect_candidates(Frame("v", "cam", m["sequence"], t, "sim_time", rgb)):
        x0, y0, x1, y1 = c.bbox
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 200, 255), 1)
    if t >= t_obs:
        x0, y0, x1, y1 = obs["bbox"]
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(img, "track confirmed", (x0, max(14, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    if t_follow is not None and t >= t_follow:
        cv2.rectangle(img, (0, 0), (W - 1, H - 1), (0, 255, 255), 4)
        cv2.putText(img, "follow-up frame linked to the observation", (12, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    side = np.full((H, panel_w, 3), 24, np.uint8)
    st = state_at(t)
    if t < t_obs:
        stage = "1  SCOUTING"
    elif t < t_acc:
        stage = "2  REVIEW (Qwen3-VL-2B)"
    elif st in ("accepted", "running"):
        stage = "3  INSPECTING"
    else:
        stage = "4  FOLLOW-UP" if st == "completed" else "3  " + str(st).upper()
    y = put(side, [stage], 14, 30, 0.7, (0, 220, 255), 30)
    y = put(side, [f"sim time  {t / 1e9:7.1f} s", f"frame      {m['sequence']}"], 14, y + 6)
    stat = next((s["state"] for s in reversed(status) if s.get("timestamp_ns", t) <= t), None) if status else None
    if stat:
        put(side, [stat[:44]], 14, y)
        y += 22
    y += 14
    put(side, ["orange: detector candidates", "green: confirmed track"], 14, y, 0.45, (160, 160, 160), 18)
    y += 50
    if t >= t_obs:
        if t < t_acc:
            y = put(side, ["review running..."], 14, y, 0.6, (0, 220, 255))
        else:
            y = put(side, [f"decision: {dec['outcome']}"], 14, y, 0.7, (0, 255, 0), 28)
            y = put(side, textwrap.wrap(dec["explanation"], 46)[:9], 14, y + 4, 0.42, (210, 210, 210), 17)
    if st:
        put(side, [f"inspection request: {st}"], 14, H - 20, 0.55, (0, 255, 0) if st == "completed" else (0, 220, 255))
    ff.stdin.write(np.hstack([img, side]).tobytes())
ff.stdin.close(); ff.wait()
print(out, len(frames), "frames")
