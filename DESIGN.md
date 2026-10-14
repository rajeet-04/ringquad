# Morphing Aerial-Ground Quad: Design Spec (v0.4)

Status: layout and fold geometry agreed, pre-build. Simulation first (ROS 2 Jazzy, Gazebo Harmonic), hardware later.

Items marked **(default)** are my proposals, not yet confirmed.

## 1. Concept

A quadrotor with four arms. Each arm ends in a ring propeller: the blades (spokes) inside the ring make thrust, and the outer surface of the ring is a tire.

- **Flight:** arms spread in an X, the ring propellers spin horizontally and fly the vehicle.
- **Ground:** arms fold down and inward with a twist, so each ring propeller stands up like a car wheel facing forward. The same motors now spin the rings as wheels, and the vehicle drives like an RC car.

Springs at each arm hinge give suspension in ground mode.

## 2. Hardware layout

| Part | Count | Role |
|---|---|---|
| Rotor/wheel motors | 4 | One per arm tip. Spin the ring propeller for thrust in flight, and drive it as a wheel on the ground. |
| Fold motor | 1 | In the body, at center-of-mass height. Drives all four arm hinges through a shared linkage or gear train. |
| Hinge springs | 4 | One per arm at the body joint. Suspension in ground mode. |

## 3. Fold geometry and sizing

Computed with `geometry.py` (body frame: x forward, y left, z up; front-left arm, others mirrored).

**Chosen design: plain front-to-back hinge, 80° fold, arm 45° down on the ground, 10° camber.**

At this arm angle the hinge axis that does the "fold inward with a spin" comes out as a pure front-to-back axis (computed: -1.0, -0.004, -0.005). So each arm needs only a plain hinge, not a tilted one, which is much easier to build. A single 80° rotation turns the arm from the diagonal X pose to 45° down, and turns the ring from horizontal to an upright wheel with 10° negative camber.

Inputs: 27 cm ring outer diameter (10-inch prop + rim), 3 cm tread, 16 x 16 x 8 cm body, ring plane offset 3.5 cm from the arm (above the arm in flight, outside the body on the ground), 1 cm minimum gaps.

| Quantity | Value |
|---|---|
| Arm length (hinge to ring centre) | **12 cm** |
| Gap between rings in flight | 6.0 cm |
| Gap between front and rear rings on the ground | 6.1 cm |
| Body ground clearance | 17 cm |
| Wheelbase / track | 33 / 30.5 cm (ratio 1.09, good for tank steering) |
| CoM height on the ground | 21 cm |
| Sideways tip-over threshold | 0.73 g |

Trade-off, same 12 cm arm: lowering the arm angle on the ground lowers the body (more stable, less clearance) but makes the hinge axis tilted.

| Arm angle down | Clearance | Tip-over | Wheelbase/track | Hinge |
|---|---|---|---|---|
| 20° | 12.7 cm | 0.87 g | 1.33 | tilted |
| 30° | 14.5 cm | 0.80 g | 1.24 | tilted |
| **45°** | **17.0 cm** | **0.73 g** | **1.09** | **plain front-to-back** |

Assumes the CoM is at hinge height and ignores arm and ring mass; recompute with real masses from CAD.

## 4. Geometry rules

- Rotor plane at center-of-mass height in flight.
- Hinges at center-of-mass height, so folding does not shift the center of mass much.
- Ring propellers do not overlap in flight.
- In ground mode, front and rear rings on each side must not touch, and the body must clear the ground.

## 5. Motors: flight vs ground

- **Same motors in both modes.** In flight they spin fast for thrust. On ground they spin slowly as wheels.
- **Spin direction changes between modes.** In flight, diagonal pairs spin opposite ways to cancel torque. On ground, both wheels on one side must spin the same way to drive forward. So at least two motors reverse direction when switching modes, and all four reverse for driving backward. Motors and controllers must be bidirectional.
- **Low-speed torque (hardware).** Drone motors and normal ESCs are poor at slow, high-torque spinning. For hardware, use motor controllers with field-oriented control and position encoders, which handle both fast flight and slow wheel driving. In simulation this is just a joint velocity/torque limit.
- Blades still spin while driving, but at wheel speeds (around 150 RPM at 2 m/s for a 10-inch ring) they make negligible thrust.

## 6. Fold mechanism requirements

- All four arms move together from one command (one motor, shared linkage).
- Holds position without continuous motor torque (worm gear, lead screw or linkage that locks at end of travel).
- **Arms locked rigid in flight; springs active only in ground mode (default).** A sprung arm in flight wobbles under thrust and upsets attitude control.

## 7. Driving and steering

- **Tank-style (skid) steering (decided):** turn by spinning the left and right wheels at different speeds. Needs no extra parts. The 1.09 wheelbase-to-track ratio keeps turning torque reasonable.

## 8. Modes

| Mode | Motors | Fold | Controller |
|---|---|---|---|
| FLY | fast, thrust and attitude | spread, locked | flight controller |
| TRANSITION_TO_GROUND | land, spin down, stop | folding | mode-switch node |
| DRIVE | slow, as wheels, direction per side | folded, springs active | ground controller (skid steer) |
| TRANSITION_TO_FLY | stopped, then spin up | unfolding, then lock | mode-switch node |
| HOLD | stopped | either | safety supervisor |

The mode-switch node gates everything: only one controller (flight or ground) commands the motors at a time.

## 9. Simulation plan (Gazebo Harmonic + ROS 2 Jazzy)

- Model: body, four arms on front-to-back revolute hinges (80° fold), four ring propellers on continuous joints, spring-damper on the hinges, ring collision shape for wheel contact.
- Fold: one command drives all four hinges (mimic joints or one controller for all four).
- Thrust: per-rotor thrust and drag torque from rotor speed (Gazebo multicopter motor model or our own plugin).
- **Controllers written in ROS 2 (default):** own flight controller and own skid-steer ground controller, with the mode-switch node between them. PX4 cannot switch vehicle type mid-mission without custom work, and an own controller is a stronger portfolio piece. Revisit PX4 for hardware.
- **Sensors (default):** IMU, depth camera, 2D LiDAR.

## 10. Demo mission (default)

Take off, fly over a wall, land, fold, drive through a low tunnel it cannot fly through, unfold, take off, and land at a goal.

## 11. Key tests

1. Hover stability with arms spread and locked.
2. Fold cycle: FLY to DRIVE and back; time per transition and no ring/body collisions during the fold.
3. Landing onto rings and spring-damper response in DRIVE.
4. Straight-line driving and skid-steer turns: slip and path error.
5. Motor direction switch between modes is correct for all four motors.
6. Fault case: one motor failure during transition, and the mode-switch response.

## 12. Findings from simulation

**Ring inertia dominates yaw.** Each ring (60 g at 12–13.5 cm radius) has about 12x the spin inertia of a plain 10-inch prop. When the mixer changes rotor speeds to make yaw torque, the reaction from spinning the rings up or down kicks the body in yaw like a reaction wheel. For a step command this kick is roughly 300x the aerodynamic drag torque the mixer plans for. With normal quad yaw gains the motors chattered between full and zero and the vehicle lost roll control. Fix: yaw gains about 50x lower than roll/pitch (attitude 2, rate 0.4). The unit-test simulator models motor lag and ring inertia, so it catches this.
- Hardware consequence: ring mass is a first-order design parameter. Lighter rings (or rings that do not spin, i.e. a fixed duct with the prop inside) give much better yaw control.

**Skid-steer turning jams in the physics engine.** DART limits tyre friction per direction (a pyramid), so a tyre sliding forward still grips fully sideways. With wheelbase/track 1.09 the robot could not turn on the spot until it broke loose in jerks. Fix in simulation: sideways tread friction 0.6, forward 1.0 (it must stay below 0.92 x forward). A yaw-rate loop on the IMU then tracks turn commands (1.01 rad/s for 1 rad/s).
- Hardware consequence: on real tyres the friction circle helps, but a wheelbase/track ratio near or above 1 still makes skid steering scrub hard. Keep the track at least as wide as the wheelbase.

**Same motors fly and drive.** One motor model per ring takes signed speed commands. Positive = upward thrust in flight; on the ground the controller maps body speed and yaw rate to signed wheel speeds. Spin direction per motor is handled in `mixer.py`.

## 13. Open questions

- Ring weight and whether the ring spins with the prop (see findings).
- Fold motor type: servo, geared brushless or lead screw.
- Failure policy when a motor fails in DRIVE.

## 14. Status and next steps

Done in simulation: model, fold, flight (take-off, waypoint + yaw, land), driving and turning, mode switching; unit and end-to-end tests. Demo course (wall + tunnel) and autonomous mission (`demo.launch.py`): fly over the wall, land, fold, drive through the tunnel with Stanley line following, unfold, fly to the goal pad. Recorded in `media/`.

The mission uses known waypoints and ground-truth odometry; it does not perceive the course yet.

Next:
1. Hinge spring-damper suspension in DRIVE (currently a stiff hold).
2. State estimation from IMU instead of ground-truth odometry.
3. Sensors (depth camera, 2D LiDAR) so the robot finds the wall and tunnel itself.
4. GPU rendering (Vulkan through Podman's libkrun to Metal) to speed up the GUI and recording.
