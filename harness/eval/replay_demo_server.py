#!/usr/bin/env python3
"""Read-only live replay demo: real clips run through the harness, with live Qwen reviews.

Each clip is decoded, passed through the detector and tracker, and every confirmed track is sent to the local
Qwen3-VL review model while the clip plays. Viewers get an MJPEG stream with overlays and a decision list.
GET only, token required (?k=TOKEN once, then a cookie). Nothing a viewer sends reaches the harness.

usage: replay_demo_server.py --model MODEL_DIR --clips DIR --token TOKEN [--port 8801] [--max-reviews 6]
Run through the harness env: uv run --project harness python harness/eval/replay_demo_server.py ...
"""
import argparse
import asyncio
import hmac
import json
import tempfile
import threading
import time
import uuid
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2

from ringquad_harness.agent import validate_decision
from ringquad_harness.model_mlx import MLXReviewBackend
from ringquad_harness.pipeline import PerceptionPipeline
from ringquad_harness.video import iter_video

CLIPS = [  # id, label shown on the page
    ("mGE_94bP_Vg", "Paddy aerial: dry patch"),
    ("1bQMEkWDsQs", "Corn close-ups (Ag PhD)"),
    ("vNN8RLy1qBo", "Drone scouting vlog (iowANFarmer)"),
    ("WXnFXSjcmWE", "Healthy wheat, low flight"),
]
COLORS = {"inspect": (60, 60, 255), "monitor": (0, 200, 255), "insufficient_evidence": (180, 180, 180),
          "failed": (255, 0, 255), "pending": (0, 255, 0)}

PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>RingQuad harness: live review on real footage</title>
<style>body{margin:0;background:#111;color:#eee;font:15px system-ui}main{max-width:1100px;margin:auto;padding:12px}
.row{display:flex;gap:14px;flex-wrap:wrap}img{width:100%;max-width:720px;background:#222}
.side{flex:1;min-width:300px}h1{font-size:18px}small{color:#999}.clip{color:#fc0;font-size:20px;margin:0 0 6px}
.r{border-top:1px solid #333;padding:6px 0;font-size:13px;color:#ccc}.inspect{color:#f66}.monitor{color:#fc0}
.insufficient_evidence{color:#aaa}.failed{color:#f6f}.pending{color:#4f4}b{font-size:14px}</style>
<main><h1>RingQuad harness: live review on real footage <small>local Qwen3-VL-2B, no cloud</small></h1><div class=row>
<img src="mjpeg" alt="video"><div class=side><p class=clip id=clip>starting</p><p id=stats></p><div id=list></div>
<small>Boxes: green = reviewing, red = inspect, yellow = monitor, grey = insufficient evidence, magenta = review failed.
Third-party footage, internal demo only. No accuracy claim: there is no labelled ground truth.</small></div></div></main>
<script>async function t(){try{const s=await (await fetch('state')).json();
clip.textContent=s.clip||'';stats.textContent='frame '+(s.frame||0)+' | tracks '+(s.tracks||0)+' | reviewed '+(s.reviewed||0);
list.innerHTML=(s.reviews||[]).slice().reverse().map(r=>'<div class=r><b class="'+r.outcome+'">'+r.outcome+'</b> '+
r.clip+' t='+r.t+'s<br>'+(r.explanation||'')+'</div>').join('')}catch(e){clip.textContent='offline'}setTimeout(t,1000)}t()</script>"""


class Demo:
    def __init__(self, model: Path, clips: Path, max_reviews: int, width: int):
        self.clips, self.max_reviews, self.width = clips, max_reviews, width
        self.backend = MLXReviewBackend(model, timeout=90.0)
        self.jpeg: bytes | None = None
        self.seq = 0
        self.cond = threading.Condition()
        self.snap: dict = {"clip": "loading model", "reviews": [], "frame": 0, "tracks": 0, "reviewed": 0}
        self.lock = threading.Lock()
        self.busy = False
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        threading.Thread(target=self.run, daemon=True).start()

    def state(self) -> dict:
        with self.lock:
            return json.loads(json.dumps(self.snap))

    def _review(self, obs, boxes: dict, label: str, t: float) -> None:
        async def go():
            try:
                decision = validate_decision(await self.backend.review(obs, obs.evidence), obs)
                return decision.outcome, decision.explanation
            except Exception as exc:  # recorded as a failure, as in the offline runs
                return "failed", f"{type(exc).__name__}: {exc}"[:160]
        outcome, text = asyncio.run_coroutine_threadsafe(go(), self.loop).result()
        with self.lock:
            boxes[obs.observation_id]["outcome"] = outcome
            self.snap["reviews"].append({"outcome": outcome, "clip": label, "t": round(t, 1), "explanation": text})
            self.snap["reviewed"] += 1
            self.busy = False

    def run(self) -> None:
        while True:
            for clip_id, label in CLIPS:
                path = self.clips / f"{clip_id}.mp4"
                if not path.is_file():
                    continue
                with self.lock:
                    self.snap.update(clip=label, frame=0, tracks=0, reviewed=0, reviews=[])
                    self.busy = False
                self.play(path, clip_id, label)

    def play(self, path: Path, clip_id: str, label: str) -> None:
        boxes: dict = {}
        started = 0
        with tempfile.TemporaryDirectory() as tmp:
            pipeline = PerceptionPipeline(Path(tmp))
            sid = str(uuid.uuid4())
            last = time.monotonic()
            for frame in iter_video(path, sid, "demo", sample_every=5):
                for obs in pipeline.update(frame):
                    if obs.observation_id in boxes:
                        continue
                    boxes[obs.observation_id] = {"bbox": obs.bbox, "outcome": "pending", "seen": frame.sequence}
                    with self.lock:
                        self.snap["tracks"] = len(boxes)
                        free = not self.busy and started < self.max_reviews
                        if free:
                            self.busy = True
                    if free:
                        started += 1
                        threading.Thread(target=self._review, daemon=True,
                                         args=(obs, boxes, label, frame.capture_ns / 1e9)).start()
                    else:
                        boxes[obs.observation_id]["outcome"] = "skip"
                img = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
                for b in boxes.values():
                    if b["outcome"] == "skip" or frame.sequence - b["seen"] > 150:
                        continue
                    x0, y0, x1, y1 = b["bbox"]
                    cv2.rectangle(img, (x0, y0), (x1, y1), COLORS.get(b["outcome"], (0, 255, 0)), 3)
                scale = self.width / img.shape[1]
                img = cv2.resize(img, (self.width, int(img.shape[0] * scale)))
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 78])
                with self.lock:
                    self.snap["frame"] = frame.sequence
                if ok:
                    with self.cond:
                        self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                        self.cond.notify_all()
                time.sleep(max(0.0, 0.12 - (time.monotonic() - last)))
                last = time.monotonic()
            # let the last review land before the next clip
            for _ in range(120):
                with self.lock:
                    if not self.busy:
                        break
                time.sleep(0.5)

    def wait(self, last: int):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last, timeout=5)
            return self.seq, self.jpeg


def make_handler(demo: Demo, token: str, max_streams: int):
    streams = threading.BoundedSemaphore(max_streams)

    class Handler(BaseHTTPRequestHandler):
        def _ok(self, query) -> bool:
            cookie = SimpleCookie(self.headers.get("Cookie", ""))
            given = (query.get("k") or [None])[0] or (cookie["k"].value if "k" in cookie else "")
            return hmac.compare_digest(str(given), token)

        def do_GET(self):  # noqa: N802
            url = urlparse(self.path)
            if not self._ok(parse_qs(url.query)):
                self.send_error(401)
                return
            if url.path == "/":
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Set-Cookie", f"k={token}; Path=/; HttpOnly; SameSite=Strict; Secure")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif url.path == "/state":
                body = json.dumps(demo.state()).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif url.path == "/mjpeg":
                if not streams.acquire(blocking=False):
                    self.send_error(503, "too many viewers")
                    return
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    last = -1
                    while True:
                        last, jpeg = demo.wait(last)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    streams.release()
            else:
                self.send_error(404)

        def _deny(self):
            self.send_error(405)

        do_POST = do_PUT = do_DELETE = do_PATCH = _deny  # noqa: N815

        def log_message(self, *_):
            pass

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--clips", type=Path, required=True)
    ap.add_argument("--token", required=True)
    ap.add_argument("--port", type=int, default=8801)
    ap.add_argument("--max-reviews", type=int, default=6)
    ap.add_argument("--max-viewers", type=int, default=6)
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--only", help="play just this clip id")
    args = ap.parse_args()
    if len(args.token) < 16:
        raise SystemExit("token must be at least 16 characters")
    if args.only:
        CLIPS[:] = [c for c in CLIPS if c[0] == args.only]
        if not CLIPS:
            raise SystemExit(f"unknown clip id: {args.only}")
    demo = Demo(args.model, args.clips, args.max_reviews, args.width)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(demo, args.token, args.max_viewers))
    print(f"replay demo on 127.0.0.1:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
