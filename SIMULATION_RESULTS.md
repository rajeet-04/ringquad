# Isolated simulation validation — 2026-11-13

Branch: `experiment/tunnel-control`, follow-up to `64a101b`.
Environment: existing Podman image, ROS 2 Jazzy and Gazebo Harmonic; realistic motors, noisy IMU, rounded contact, DART/Bullet. The robot mechanism, torque and friction limits are unchanged by this follow-up.

## Fixes

- Every validation run has a unique Gazebo partition and a locked ROS domain. The harness removes the entire process group even when the launch wrapper exits first. Four orphaned Gazebo servers were discovered during diagnosis; older unisolated repeats cannot reliably distinguish dynamics from command/odometry cross-talk.
- Gyro rate is filtered with a 50 ms time constant. The yaw integrator no longer loses half its accumulated correction whenever sensor noise changes the error sign; saturation anti-windup remains. A fixed-seed regression with 0.05 rad/s gyro noise and a 0.05 rad/s target fails the previous controller (mean 0.0386 rad/s) and passes the corrected controller within 0.01 rad/s.
- A ground-alignment timeout stops the wheels and enters the existing aerial retry instead of aborting immediately. The next landing heading compensates the measured folded heading error. Three landing attempts remain the limit.
- Flying to a waypoint now requires heading convergence within 1 degree as well as position and velocity convergence, so landing cannot begin before the requested alignment is reached.
- The final ground-alignment allowance is 60 simulated seconds. The tunnel entry tolerance remains 1 degree and the strict cross-track abort remains 5 cm.

## Completed isolated missions

| Tread | Run | Simulated duration | Worst drive cross-track | Goal error |
|---|---|---:|---:|---:|
| mixed | 1 | 114 s | 2.3 cm | 4 cm |
| mixed | 2 | 81 s | 1.5 cm | 2 cm |
| rubber | 1 | 66 s | 3.4 cm | 2 cm |
| rubber | 2 | 66 s | 2.1 cm | 1 cm |
| omni | 1 | 219 s | 1.7 cm | 1 cm |
| omni | 2 | 100 s | 2.5 cm | 2 cm |

All six ran the filtered yaw controller and compensated recovery. The first five used a 30 s ground-alignment allowance; the last omni run exercised the final 60 s allowance. Mixed run 1 and omni run 1 exercised actual stalled-ground-alignment recovery. Omni run 1 used all three landing attempts. Longer completion times reflect bounded retries, not a relaxed drift threshold.

## Other verification

- 25 Python tests passed in the sourced container, including actual mission-generator tests for stalled alignment recovery, bounded failure, and waiting for heading before waypoint completion.
- Original `mission_test.py`: all seven checks passed on isolated rubber tread. Takeoff error 1.6 cm; waypoint error 1.9 cm, yaw error 0.0 degrees; forward speed 0.50 m/s with negligible sideways speed; landing, folding and unfolding passed.
- The turn-request regression kept 0.37 m/s forward speed, but achieved only 0.01 rad/s yaw for a 0.3 rad/s request. It verifies absence of drive stall, not accurate or arbitrary skid steering.
- A forced 2 s harness timeout correctly returned failure. After all tests, process inspection found no remaining Gazebo servers, RingQuad controllers, or mission nodes.

Host logs: `/private/tmp/ringquad-resolution-results/`, containing `mixed-1.log`, `mixed-2.log`, `rubber-1.log`, `rubber-2.log`, `omni-1.log`, `omni-2.log`, and `regression.log`.

Reproduce inside the sourced ROS workspace:

```bash
python3 -m pytest -q src/ringquad_control/test
for tread in mixed rubber omni; do
  python3 scripts/demo_test.py --runs 2 --tread "$tread" --timeout 300 || exit 1
done
```

## Limits

The complete wall/tunnel mission passes for all three treads in the sampled runs. Ground steering remains physically weak and the mission may need aerial alignment retries; these results do not establish requested yaw-rate tracking or every initial condition. Rounded contact approximates tread, not individual rollers. Hardware still requires a physical steering and thrust test rig. The explicit-torque motor prototype was rejected after a flight regression and is not included in the active source changes.

## Added-hardware variant: front steering with positive flight lock

The optional `steering:=front tread:=rubber` variant adds two steering knuckles, an ideal positive flight lock, Ackermann allocation, extra-mass thrust compensation and CoM-aware flight torque allocation. `steering:=fixed` remains the default. Hardware assumptions and commands are in STEERING.md.

| Final locked variant | Result |
|---|---|
| Complete mission 1 | 68 simulated seconds; worst drive cross-track 3.2 cm; goal error 2 cm |
| Complete mission 2 | 68 simulated seconds; worst drive cross-track 3.1 cm; goal error 2 cm |
| Left arc at 0.4 m/s | Requested +0.2 rad/s; measured +0.20486 rad/s |
| Right arc at 0.4 m/s | Requested -0.2 rad/s; measured -0.20597 rad/s |
| Python suite | 31 passed, including steering geometry, motor/joint limits, tunnel envelope and CoM hover balance |
| Model checks | fixed 1.68 kg; front steering 1.72 kg; two steering joints; valid SDF conversion |
| Original mechanism regression | Fixed-wheel rubber full mission passed after the optional mechanism was added |

The two full missions used isolated ROS domains and unique Gazebo partitions. No Gazebo servers or controllers remained after cleanup. Logs were captured under `/private/tmp/ringquad-front-steering-results/` during validation. That temporary directory is no longer present at commit time; the table records the outcomes observed during the runs.

Rejected intermediate versions are retained only in diagnostic logs: servo hold without a positive flight lock produced roll oscillations and, on one repeat, a flight crash after successful driving. These runs are not counted as passing. The final lock restricts steering travel to zero until the arm fold exceeds 75 degrees. The node centres steering before unfolding. The lock's structural strength, compliance and engagement energy are not simulated and require hardware testing.

Normal front-steered flight waypoint allowance is 30 simulated seconds; fixed steering retains 20 seconds. Both still require the original position/velocity tolerances and a 1 degree heading tolerance. The 5 cm tunnel drift abort remains unchanged. One bridge process in a completed test exited with a shutdown-only segmentation fault; this does not invalidate the preceding mission completion, but middleware shutdown should not be described as uniformly clean.
