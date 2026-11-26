#!/usr/bin/env bash
# Stop the tunnel, dashboard, any running scenario and the sim container.
DEMO_DIR="${DEMO_DIR:-$HOME/.cache/ringquad-demo}"
for name in tunnel server; do
  [ -f "$DEMO_DIR/$name.pid" ] && kill "$(cat "$DEMO_DIR/$name.pid")" 2>/dev/null
  rm -f "$DEMO_DIR/$name.pid"
done
pkill -f scouting_sim_test.py 2>/dev/null
rm -f "$DEMO_DIR/link.txt"
podman stop ringquad-harness >/dev/null 2>&1
echo "demo stopped"
