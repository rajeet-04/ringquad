#!/usr/bin/env bash
# Build the dev image (first time) and start the container.
# Desktop in the browser: http://localhost:6080
set -euo pipefail
cd "$(dirname "$0")"

IMAGE=ringquad-dev
NAME=ringquad

if ! podman image exists "$IMAGE"; then
  podman build -t "$IMAGE" -f Containerfile .
fi

if podman container exists "$NAME"; then
  podman start "$NAME"
else
  podman run -d --name "$NAME" \
    -p 6080:80 \
    --shm-size=1g \
    --sysctl net.ipv4.ip_unprivileged_port_start=0 \
    -v "$PWD:/home/ubuntu/ringquad_ws" \
    "$IMAGE"
fi

# Existing containers receive the same pinned engine as newly built images.
podman exec -u ubuntu "$NAME" bash -lc 'cd /home/ubuntu/ringquad_ws && bash hardware-blockers/physics/build.sh build/ringquad-physics'

# The desktop's screensaver locks the session behind a password; turn it off.
sleep 5
podman exec -u ubuntu -e DISPLAY=:1 "$NAME" bash -c '
  pid=$(pgrep -u ubuntu -f "[m]ate-session" | head -1)
  [ -n "$pid" ] && export $(tr "\0" "\n" < /proc/$pid/environ | grep ^DBUS_SESSION_BUS_ADDRESS=)
  gsettings set org.mate.screensaver lock-enabled false
  gsettings set org.mate.screensaver idle-activation-enabled false
  pkill -u ubuntu -f "[m]ate-screensaver"' || true

echo "Desktop: http://localhost:6080"
echo "Shell:   podman exec -it -u ubuntu $NAME bash"
