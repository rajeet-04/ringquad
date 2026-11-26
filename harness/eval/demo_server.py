#!/usr/bin/env python3
"""Read-only demo dashboard: camera with detector overlay plus the runner's live state.

Serves GET only. It never forwards anything to the bridge, so a viewer cannot command the
simulated drone. Requires the token (?k=TOKEN once, then a cookie) so the link is not guessable.

usage: demo_server.py --bridge http://127.0.0.1:8765/stream --state STATE.json --token TOKEN [--port 8800]
Run through the harness env: uv run --project harness python harness/eval/demo_server.py ...
"""
import argparse
import hmac
import json
import threading
import time
import urllib.request
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from ringquad_harness.live import iter_records
from ringquad_harness.perception import detect_candidates
from ringquad_harness.schemas import Frame

PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>RingQuad scouting demo</title>
<style>body{margin:0;background:#111;color:#eee;font:15px system-ui}main{max-width:1000px;margin:auto;padding:12px}
.row{display:flex;gap:14px;flex-wrap:wrap}img{width:100%;max-width:640px;background:#222}
.side{flex:1;min-width:280px}h1{font-size:18px}.stage{color:#fc0;font-size:22px;margin:0 0 8px}
.ok{color:#4f4}pre{white-space:pre-wrap;color:#ccc;font:13px system-ui}small{color:#999}</style>
<main><h1>RingQuad scouting demo <small>local AI, simulated drone</small></h1><div class=row>
<img src="mjpeg" alt="camera"><div class=side><p class=stage id=stage>connecting</p>
<p>decision: <b id=dec>-</b> <small id=att></small></p><pre id=exp></pre>
<p>inspection request: <b id=req>-</b></p><p>result: <b id=res>-</b></p>
<small>orange boxes: detector candidates. green box: confirmed track. Read-only view.</small></div></div></main>
<script>async function t(){try{const s=await (await fetch('state')).json();
stage.textContent=(s.stage||'idle').toUpperCase();dec.textContent=s.decision||'-';
att.textContent=s.attempt?'(observation '+s.attempt+')':'';exp.textContent=s.explanation||'';
req.textContent=(s.request||[]).join(' > ')||'-';res.textContent=s.result||'-';
res.className=s.result==='closed'?'ok':''}catch(e){stage.textContent='offline'}setTimeout(t,1000)}t()</script>"""


class Hub:
    """Pulls the bridge stream, draws overlays, keeps the newest JPEG."""

    def __init__(self, url: str, state_path: str):
        self.url, self.state_path = url, state_path
        self.jpeg: bytes | None = None
        self.seq = 0
        self.cond = threading.Condition()
        threading.Thread(target=self.run, daemon=True).start()

    def state(self) -> dict:
        try:
            with open(self.state_path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {"stage": "idle"}

    def run(self) -> None:
        for kind, item in iter_records(lambda: urllib.request.urlopen(self.url, timeout=10), reconnect_delay=1.0):
            if kind != "record" or item[0].get("kind") != "frame":
                continue
            meta, payload = item
            img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            frame = Frame("demo", "cam", int(meta["sequence"]), int(meta["capture_ns"]), str(meta["clock_domain"]),
                          img[:, :, ::-1].copy())
            for c in detect_candidates(frame):
                x0, y0, x1, y1 = c.bbox
                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 200, 255), 1)
            st = self.state()
            if st.get("track_bbox") and st.get("stage") in ("reviewing", "inspecting", "done"):
                x0, y0, x1, y1 = st["track_bbox"]
                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.cond:
                    self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                    self.cond.notify_all()

    def wait(self, last: int) -> tuple[int, bytes | None]:
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last, timeout=5)
            return self.seq, self.jpeg


def make_handler(hub: Hub, token: str, max_streams: int):
    streams = threading.BoundedSemaphore(max_streams)

    class Handler(BaseHTTPRequestHandler):
        def _authorized(self, query: dict) -> bool:
            cookie = SimpleCookie(self.headers.get("Cookie", ""))
            given = (query.get("k") or [None])[0] or (cookie["k"].value if "k" in cookie else "")
            return hmac.compare_digest(str(given), token)

        def do_GET(self):  # noqa: N802
            url = urlparse(self.path)
            query = parse_qs(url.query)
            if not self._authorized(query):
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
                body = json.dumps(hub.state()).encode()
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
                        last, jpeg = hub.wait(last)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
                        time.sleep(0.05)
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
    ap.add_argument("--bridge", default="http://127.0.0.1:8765/stream")
    ap.add_argument("--state", required=True)
    ap.add_argument("--token", required=True)
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument("--max-viewers", type=int, default=6)
    args = ap.parse_args()
    if len(args.token) < 16:
        raise SystemExit("token must be at least 16 characters")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(Hub(args.bridge, args.state), args.token, args.max_viewers))
    print(f"dashboard on 127.0.0.1:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
