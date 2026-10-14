# RingQuad

A quadrotor whose ring propellers fold down into wheels. Flies over obstacles, drives through spaces it cannot fly through. ROS 2 Jazzy + Gazebo Harmonic. Design: [DESIGN.md](DESIGN.md).

![Demo: fly over the wall, land, fold into wheels, drive through the tunnel, unfold, fly to the goal](media/demo.gif)

The demo course ([worlds/demo.sdf](src/ringquad_description/worlds/demo.sdf)): a 1.5 m wall to fly over, then a tunnel 0.5 m wide and 0.4 m high. Spread out the drone is about 0.6 m wide and cannot fly through; folded it is about 0.36 m wide and drives through. The whole run is autonomous: 46–51 s, worst sideways error in the tunnel 0.1–1.7 cm, landing within 0–8 cm of the goal pad centre over four runs. GIF at 2x; [demo.mp4](media/demo.mp4) is real time.

## Run (macOS or Linux with Podman)

```bash
./run.sh
```

Open http://localhost:6080 for the desktop. In a terminal there:

```bash
cd ~/ringquad_ws && colcon build --symlink-install && source install/setup.bash

# Gazebo + RViz + controller
ros2 launch ringquad_control sim.launch.py

# RViz only, fold the arms with sliders
ros2 launch ringquad_description display.launch.py
```

```bash
# demo course + autonomous mission
ros2 launch ringquad_control demo.launch.py
```

From the Mac, `scripts/sim_restart.sh` (re)starts the simulation in the running container; it passes its arguments to `sim.launch.py` (for example `scripts/sim_restart.sh world:=demo`).

## Commands

The controller node is the only thing that commands the motors and the fold. Pick a mode:

```bash
# take off to 1 m above the current spot
ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: fly}"

# fly to a point (odom frame) and heading
ros2 topic pub --once /ringquad/goal geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 1.0, y: 0.5, z: 1.5}, orientation: {z: 0.707, w: 0.707}}}"

# land; switches to idle on touchdown
ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: land}"

# on the ground only: fold into wheels and drive with /cmd_vel
ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: drive}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}"

# motors off, arms unfolded
ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: idle}"
```

## Tests

```bash
# unit tests: mixer, flight controller (rigid body with motor lag and ring inertia), yaw-rate loop
cd ~/ringquad_ws/src/ringquad_control && python3 -m pytest -q test

# end to end against a freshly started sim.launch.py
python3 ~/ringquad_ws/scripts/mission_test.py
```

`mission_test.py` takes off, flies to a point while turning 90 deg, lands, folds into wheels, drives at 0.5 m/s, turns on the spot at 1 rad/s and unfolds. Current results:

| Check | Result |
|---|---|
| Take off to 1 m | 1.0 cm error |
| Fly to (1, 0.5, 1.5) m + yaw 90 deg | 2.2 cm, 0.1 deg error |
| Land and disarm | on the ground, idle |
| Fold into wheels | 80.0 deg, all four hinges |
| Drive at 0.5 m/s | 0.50 m/s, no sideways drift |
| Turn on the spot at 1 rad/s | 1.01 rad/s |
| Unfold | 0.0 deg |

## Layout

| Path | What |
|---|---|
| `src/ringquad_description/urdf/ringquad.urdf.xacro` | Robot model, motor models, fold controllers, sensors |
| `src/ringquad_description/worlds/` | `ringquad.sdf` empty world, `demo.sdf` demo course (1 ms physics, IMU system) |
| `src/ringquad_control/ringquad_control/` | `params.py` vehicle constants, `mixer.py` allocation and skid steering, `flight.py` geometric controller, `ground.py` yaw-rate loop and line following, `node.py` ROS node and modes, `mission.py` demo mission |
| `media/` | Demo video and GIF |
| `scripts/mission_test.py` | End-to-end test |
| `scripts/flight_log.py`, `scripts/motor_probe.py` | Diagnostics: log a take-off, hold a motor speed |
| `geometry.py` | Fold geometry and sizing calculator |
| `DESIGN.md` | Design spec |
