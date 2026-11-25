#!/usr/bin/env python3
"""Record the camera bridge stream (frames, poses, status) to a folder. Stop with SIGINT/SIGTERM."""
import json, signal, sys, urllib.request
from pathlib import Path

from ringquad_harness.live import iter_records

out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
(out / "frames").mkdir(exist_ok=True)
stop = False
signal.signal(signal.SIGTERM, lambda *_: globals().update(stop=True))
signal.signal(signal.SIGINT, lambda *_: globals().update(stop=True))
index = open(out / "index.jsonl", "w")
for kind, item in iter_records(lambda: urllib.request.urlopen(sys.argv[1], timeout=10), reconnect_delay=0.5):
    if stop:
        break
    if kind != "record":
        continue
    meta, payload = item
    if meta.get("kind") == "frame":
        name = f"frames/{meta['sequence']:06d}.jpg"
        (out / name).write_bytes(payload)
        meta = {**meta, "file": name}
    index.write(json.dumps(meta) + "\n"); index.flush()
