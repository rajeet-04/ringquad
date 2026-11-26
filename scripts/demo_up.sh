#!/usr/bin/env bash
# Start the remote demo: sim container, read-only dashboard, Cloudflare quick tunnel, then the scenario.
# usage: scripts/demo_up.sh MODEL_DIR        (or set RINGQUAD_MODEL)  [NORUN=1 skips the scenario, NOTUNNEL=1 skips the tunnel]
# Prints the shareable link. Stop everything with scripts/demo_down.sh.
set -euo pipefail
MODEL="${1:-${RINGQUAD_MODEL:?give the model directory as an argument or set RINGQUAD_MODEL}}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEMO_DIR="${DEMO_DIR:-$HOME/.cache/ringquad-demo}"
CONTAINER=ringquad-harness
PORT=8800
mkdir -p "$DEMO_DIR"
[ -d "$MODEL" ] || { echo "model directory not found: $MODEL" >&2; exit 1; }
if [ -f "$DEMO_DIR/server.pid" ] && kill -0 "$(cat "$DEMO_DIR/server.pid")" 2>/dev/null; then
  echo "already running; run scripts/demo_down.sh first" >&2; exit 1
fi

podman machine list --format '{{.Running}}' | grep -q true || podman machine start
podman start "$CONTAINER" >/dev/null
podman exec -u ubuntu -d "$CONTAINER" bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/ringquad_ws/install/setup.bash && ros2 launch ringquad_control demo.launch.py mission:=false camera:=true world:=scouting gui:=false rviz:=false > /tmp/demo.log 2>&1'
echo "waiting for the sim camera..."
for _ in $(seq 1 60); do
  curl -s --max-time 2 http://127.0.0.1:8765/health | grep -q '"calibrated": true' && break
  sleep 3
done
curl -s --max-time 2 http://127.0.0.1:8765/health | grep -q '"calibrated": true' || { echo "camera bridge never calibrated" >&2; exit 1; }

TOKEN="$(openssl rand -hex 16)"
echo '{"stage":"ready"}' > "$DEMO_DIR/state.json"
nohup uv run --project "$ROOT/harness" python "$ROOT/harness/eval/demo_server.py" \
  --state "$DEMO_DIR/state.json" --token "$TOKEN" --port "$PORT" > "$DEMO_DIR/server.log" 2>&1 &
echo $! > "$DEMO_DIR/server.pid"
for _ in $(seq 1 30); do grep -q "dashboard on" "$DEMO_DIR/server.log" && break; sleep 1; done

if [ -n "${NOTUNNEL:-}" ]; then
  echo "LOCAL ONLY (no tunnel): http://127.0.0.1:$PORT/?k=$TOKEN"
  [ -z "${NORUN:-}" ] && RINGQUAD_MODEL="$MODEL" "$ROOT/scripts/demo_run.sh"
  exit 0
fi
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

if [ -z "${NORUN:-}" ]; then
  RINGQUAD_MODEL="$MODEL" "$ROOT/scripts/demo_run.sh"
fi
