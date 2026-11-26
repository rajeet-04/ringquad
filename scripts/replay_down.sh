#!/usr/bin/env bash
# Stop the replay demo: tunnel first, then the server.
DEMO_DIR="${DEMO_DIR:-$HOME/.cache/ringquad-replay}"
for n in tunnel server; do
  [ -f "$DEMO_DIR/$n.pid" ] && kill "$(cat "$DEMO_DIR/$n.pid")" 2>/dev/null
  rm -f "$DEMO_DIR/$n.pid"
done
rm -f "$DEMO_DIR/link.txt"
echo "replay demo stopped"
