#!/usr/bin/env bash
# Run the success scenario once against the running demo (progress shows on the dashboard).
set -euo pipefail
MODEL="${1:-${RINGQUAD_MODEL:?give the model directory as an argument or set RINGQUAD_MODEL}}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEMO_DIR="${DEMO_DIR:-$HOME/.cache/ringquad-demo}"
OUT="$DEMO_DIR/run-$(date +%H%M%S)"
nohup python3 "$ROOT/scripts/scouting_sim_test.py" --model-path "$MODEL" --out "$OUT" --scenario success \
  --attempts 20 --state-file "$DEMO_DIR/state.json" > "$OUT.log" 2>&1 &
echo "scenario started (log: $OUT.log)"
