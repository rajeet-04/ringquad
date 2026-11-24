# Simulated inspection loop traces

Recorded with `scripts/scouting_sim_test.py` against Gazebo (world `scouting`, Podman container `ringquad-harness`)
and Qwen3-VL-2B-Instruct 4-bit on the M4. Each `*.jsonl` is the `inspection.jsonl` the `inspect` command wrote.

| file | scenario | request states | result |
| --- | --- | --- | --- |
| success.jsonl | hover, observe, review, inspect | accepted, running, completed | closed: closer frame linked to the observation (`success-followup.jpg`, 0.76 m vs 1.04 m) |
| stale.jsonl | evidence pose older than 30 s | rejected ("request pose is too old") | no motion command published |
| interrupted.jsonl | `land` commanded while running | accepted, running, failed ("controller left fly mode") | failed, setpoints stop |

Not covered in simulation: lost odometry feedback (the bridge clock and odometry share one process, so they stop
together); it is unit-tested in `src/ringquad_inspection_bridge/test/test_inspection.py`.

The model said `monitor` or returned invalid output for many observations of the rendered patch; the script observes
again until one gets an `inspect` decision. Treat this as an integration check, not as model accuracy.
