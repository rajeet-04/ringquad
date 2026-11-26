#!/usr/bin/env bash
# Start the real-footage replay demo behind a Cloudflare quick tunnel. Stop with scripts/replay_down.sh.
# usage: scripts/replay_up.sh MODEL_DIR CLIPS_DIR        [NOTUNNEL=1 for local only]
set -euo pipefail
MODEL="${1:?model dir}"; CLIPS="${2:?clips dir}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEMO_DIR="${DEMO_DIR:-$HOME/.cache/ringquad-replay}"; PORT=8801
mkdir -p "$DEMO_DIR"
if [ -f "$DEMO_DIR/server.pid" ] && kill -0 "$(cat "$DEMO_DIR/server.pid")" 2>/dev/null; then
  echo "already running; run scripts/replay_down.sh first" >&2; exit 1
fi
TOKEN="$(openssl rand -hex 16)"
nohup uv run --project "$ROOT/harness" python "$ROOT/harness/eval/replay_demo_server.py" \
  --model "$MODEL" --clips "$CLIPS" --token "$TOKEN" --port "$PORT" > "$DEMO_DIR/server.log" 2>&1 &
echo $! > "$DEMO_DIR/server.pid"
for _ in $(seq 1 120); do grep -q "replay demo on" "$DEMO_DIR/server.log" && break; sleep 1; done
grep -q "replay demo on" "$DEMO_DIR/server.log" || { echo "server failed; see $DEMO_DIR/server.log" >&2; exit 1; }
if [ -n "${NOTUNNEL:-}" ]; then echo "LOCAL: http://127.0.0.1:$PORT/?k=$TOKEN"; exit 0; fi
: > "$DEMO_DIR/tunnel.log"
nohup cloudflared tunnel --no-autoupdate --url "http://127.0.0.1:$PORT" > "$DEMO_DIR/tunnel.log" 2>&1 &
echo $! > "$DEMO_DIR/tunnel.pid"
URL=""
for _ in $(seq 1 30); do
  URL="$(grep -o 'https://[a-z0-9-]*\.trycloudflare\.com' "$DEMO_DIR/tunnel.log" | head -1 || true)"
  [ -n "$URL" ] && break; sleep 1
done
[ -n "$URL" ] || { echo "tunnel did not come up; see $DEMO_DIR/tunnel.log" >&2; exit 1; }
echo "$URL/?k=$TOKEN" > "$DEMO_DIR/link.txt"; chmod 600 "$DEMO_DIR/link.txt"
echo "LINK: $URL/?k=$TOKEN"
