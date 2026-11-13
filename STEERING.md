# Optional front steering

**Current simulation design:** independent lightweight propellers and wheel drives, upright 90° fold, 100 mm ring-axis standoff, rear wheel carriers, front steering with 25° stops, physical crowned rims and self-collision enabled. Nominal mass budget is 1.98 kg. See [REDESIGN.md](REDESIGN.md) for the mechanism and regression evidence. Earlier shared-motor results below are historical; hardware qualification remains open.

**Hardware-validation update:** deeper checks found chassis/rim interference and strong contact-model sensitivity. The successful controller demos are not hardware qualification. See [HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md) before using this model as a build specification.

Run the added-hardware variant inside the built ROS workspace:

```bash
ros2 launch ringquad_control demo.launch.py steering:=front tread:=rubber
python3 scripts/demo_test.py --runs 2 --steering front --tread rubber --timeout 300
```

The redesigned default is `steering:=front`; `steering:=fixed` selects the original steering topology. Front steering is intended for rubber tread; low lateral friction on omni front wheels reduces the benefit of steering them.

## Mechanism and assumptions

Two 20 g steering knuckles pivot the front ring/motor assemblies. Their axes are vertical at the revised 90 degree fold, with upright wheels. Steering travel is limited to 25 degrees in the ground plane; the knuckle joint limit includes camber compensation. Each servo has an assumed 1 Nm torque and 1.5 rad/s joint-speed limit. These are simulation sizing assumptions, not a selected or validated hardware component.

For flight, the controller commands both steering joints to zero and verifies centring before unfolding. A Gazebo system models a positive mechanical lock: steering travel is zero below a 75 degree fold angle and released to the normal joint limits above it. Servo hold alone was rejected after flight oscillations and a crash. Flight thrust includes the extra 40 g and allocation accounts for the forward centre-of-mass shift. The simulated lock is ideal and its strength/compliance are not modelled. A real build must implement that positive lock and validate structural loads, servo heating, backlash and clearances.

## Control

Ackermann geometry uses the rear axle centre as its velocity reference, individual front-wheel steering angles, and different wheel speeds for the inner/outer paths. Curvature is limited by the inner steering joint; motor speed remains capped at 800 rad/s. At zero forward speed this variant brakes instead of claiming a pivot turn.

Ground heading alignment uses slow forward/reverse rolling corrections because a steered chassis cannot turn in place. In the narrow tunnel the mission limits curvature to a 10 degree inner steering angle, reducing the swept front-ring width. The existing 5 cm cross-track abort is retained. If alignment cannot be achieved, the bounded aerial retry remains available.

The added-hardware variant allows up to 30 simulated seconds per normal flight waypoint (fixed steering retains 20 seconds). Position, velocity and heading convergence are all required.

## Earlier validation (80-degree rounded-contact model)

The automated geometry tests cover straight driving, stopping, mirrored turns, reverse motion, a shared instantaneous turning centre, joint/motor limits, conservative tunnel width and hover balance about the shifted centre of mass.

For direct steering verification, launch an empty world and run the measured yaw-rate test in the same ROS/Gazebo environment:

```bash
ros2 launch ringquad_control sim.launch.py steering:=front tread:=rubber gui:=false rviz:=false
# In another terminal with the same sourced environment:
python3 scripts/steering_test.py
```

The final isolated ground test with the positive lock installed measured +0.2049 rad/s and -0.2060 rad/s for +0.2 and -0.2 rad/s requests at 0.4 m/s. See SIMULATION_RESULTS.md for full-mission results and limits.
