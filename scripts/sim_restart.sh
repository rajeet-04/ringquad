#!/usr/bin/env bash
# Restart the simulation inside the running container. Usage: scripts/sim_restart.sh [launch args]
set -euo pipefail
podman exec ringquad bash -c 'for p in $(ps -eo pid,args | grep -E "gz sim|gz_tools|parameter_bridge|rviz2|robot_state_publisher|ros2 launch|ringquad_control" | grep -v grep | awk "{print \$1}"); do kill -9 $p 2>/dev/null; done; true'
sleep 2
podman exec -d -u ubuntu -e DISPLAY=:1 ringquad bash -lc "source /opt/ros/jazzy/setup.bash && source ~/ringquad_ws/install/setup.bash && ros2 launch ringquad_control sim.launch.py $* > /tmp/sim.log 2>&1"
echo "sim starting, log: podman exec ringquad tail -f /tmp/sim.log"
