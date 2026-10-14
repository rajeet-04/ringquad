# RingQuad

A quadrotor whose ring propellers fold down into wheels. Flies over obstacles, drives through spaces it cannot fly through. ROS 2 Jazzy + Gazebo Harmonic. Design: [DESIGN.md](DESIGN.md).

## Run (macOS or Linux with Podman)

```bash
./run.sh
```

Open http://localhost:6080 for the desktop. In a terminal there:

```bash
cd ~/ringquad_ws && colcon build --symlink-install && source install/setup.bash

# RViz only, fold the arms with the sliders
ros2 launch ringquad_description display.launch.py

# Gazebo + RViz
ros2 launch ringquad_description gazebo.launch.py
```

## Commands (Gazebo)

```bash
# fold all four arms into wheels (80 deg) / unfold
ros2 topic pub --once /ringquad/fold std_msgs/msg/Float64 "{data: 1.396}"
ros2 topic pub --once /ringquad/fold std_msgs/msg/Float64 "{data: 0.0}"

# drive forward when folded: left rings positive, right rings negative (rad/s)
for r in fl rl; do ros2 topic pub --once /ringquad/ring_$r std_msgs/msg/Float64 "{data: 10.0}"; done
for r in fr rr; do ros2 topic pub --once /ringquad/ring_$r std_msgs/msg/Float64 "{data: -10.0}"; done
```

## Layout

| Path | What |
|---|---|
| `src/ringquad_description/urdf/ringquad.urdf.xacro` | Robot model and Gazebo plugins |
| `src/ringquad_description/launch/` | `display.launch.py` (RViz), `gazebo.launch.py` (Gazebo + RViz) |
| `geometry.py` | Fold geometry and sizing calculator |
| `DESIGN.md` | Design spec |
