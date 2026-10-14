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

echo "Desktop: http://localhost:6080"
echo "Shell:   podman exec -it -u ubuntu $NAME bash"
