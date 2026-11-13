# Simulation validation — 2026-11-13

Branch: `experiment/tunnel-control` (working-tree changes).
Environment: existing Podman `ringquad-dev` image, ROS 2 Jazzy, Gazebo Harmonic.

Final demo configuration: realistic torque-limited motors and noisy IMU, rounded convex tread collision mesh, DART dynamics with Bullet collision detection, omni tread (lateral mu=0.05, longitudinal mu=1.0), active ground heading alignment within 1 degree, unchanged 5 cm tunnel abort limit.

| Check | Observed result |
|---|---|
| ROS packages build | Both packages pass |
| Python unit suite in container | 21 passed |
| Generated URDF/SDF | rubber, mixed, omni convert successfully |
| Fresh full mission 1 | 74 simulated seconds, 1.3 cm worst drive cross-track, 1 cm goal error |
| Fresh full mission 2 | 65 simulated seconds, 1.3 cm worst drive cross-track, 1 cm goal error |
| Existing mission_test.py | All seven checks pass: takeoff, waypoint/yaw, land, fold, drive, no stall on turn, unfold |

Regression details: takeoff error 1.5 cm; waypoint error 1.9 cm with 0.1 degree yaw error; forward speed 0.50 m/s with negligible sideways speed. Achieved yaw during the turn request was only 0.01 rad/s. This test verifies continued motion, not useful steering bandwidth.

Logs: `/private/tmp/ringquad-final-results/run-1.log`, `run-2.log`, and `regression.log` on the host. Shutdown signal messages after mission completion come from the test harness terminating the completed world.

Earlier diagnostic runs included one successful rounded omni run without active alignment, one failed repeat with a 7 degree initial heading error, and a reduced-longitudinal-friction experiment. The two final runs above restore longitudinal mu=1.0 and include active alignment. The reduced-friction experiment is not part of the final model.

Reproduce inside the container, after sourcing ROS and the built workspace:

```bash
python3 scripts/demo_test.py --runs 3 --tread omni
```

Mixed and rubber remain selectable but their tunnel steering is unresolved. The rounded collision approximates a tread profile rather than individual omni rollers. These simulation results do not validate physical hardware or every initial condition.
