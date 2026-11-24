# RingQuad Agricultural AI Harness Implementation Plan

> **For agentic workers:** Use Superpowers’ executing-plans skill or subagent-driven-development skill to implement and verify one task at a time. Checkboxes track execution.

**Goal:** Run agricultural scouting on the M4 from recorded video, then connect the same harness to RingQuad in simulation and demonstrate a closer inspection.

**Architecture:** Add a standalone Python package to the existing RingQuad repository. It ingests video continuously, tracks candidates, submits selected evidence to local models, and records validated decisions. A ROS 2 adapter supplies simulated frames and telemetry and handles inspection requests through the existing controller.

**Tech stack:** Python 3.12, uv, pytest, PyAV, OpenCV, Pydantic AI, MLX-VLM as the first runtime candidate, ROS 2 Jazzy, Gazebo Harmonic, Podman.

**Spec:** The [implementation scope and contracts](#implementation-scope-and-contracts) below summarize the prior [RingQuad assessment](thread://01a20d3b-996c-7f43-9591-9835a2e7488f?hostId=local).

## Global constraints

- Develop in the existing RingQuad repository on proposed branch `experiment/agri-ai-harness`, starting at `experiment/tunnel-control`. The branch baseline inspected for this plan is `ca8d05d1844af684c39691f0c62af2d5da731dce`.
- Keep core replay, perception, and reasoning runnable on the M4 without ROS. Keep ROS dependencies in the simulation adapter and its container.
- First perception scope is one visible symptom, a persistent canopy discoloration. Record visible evidence and uncertainty. Disease classification, training a specialist model, and physical robot commissioning are outside the first milestone.
- Use bounded queues in live mode. Replay evaluation uses clip timestamps and a deterministic path that processes all sampled frames. Store media and model weights outside Git, with fixture manifests and checksums in Git.
- Use the existing controller for simulated motion. Robot-bound requests require a fresh pose, a resolved target in a supported frame, and an eligible controller state.

## Review focus

- Variable frame timing and skipped frames must preserve original capture time. Task 1.
- Brief occlusion or a single glint must avoid duplicate proposals for one continuously observed patch. Task 2.
- A schema-valid model output can cite evidence from another track. Task 3.
- Missing or stale localization must preserve image-space observations and block robot-bound requests. Task 4.
- A repeated request or a lost feedback signal must avoid duplicate motion and reach a terminal status. Task 5.

## Implementation scope and contracts

| Record | Required information |
|---|---|
| `Frame` | Session ID, camera ID, integer sequence, capture time in nanoseconds, clock domain, image, and optional calibration ID. |
| `PoseSample` | Timestamp and clock domain, coordinate frame ID, position, orientation, and source. Vehicle mode and fault status retain their own sample times. |
| `Observation` | Stable ID within the session, first and last capture timestamps, image-space region, and evidence frame IDs. Optional position carries a named coordinate frame and a location source. |
| `Decision` | Observation ID, outcome of `inspect`, `monitor`, or `insufficient_evidence`, evidence frame IDs, explanation, and requested view for `inspect`. |
| `InspectionRequest` | Request ID, observation ID, evidence IDs, requested view, target pose and its coordinate frame, plus timestamps needed for freshness checks. |
| `InspectionStatus` | Request ID, state of `accepted`, `running`, `completed`, `rejected`, or `failed`, timestamp, and reason where applicable. |

Use `harness/` for the uv project and its `ringquad_harness` Python package. Use `src/ringquad_inspection_bridge/` for ROS code, with camera changes scoped to `src/ringquad_description/` and an opt-in argument in `src/ringquad_control/launch/demo.launch.py`. Keep the offline CLI and simulation smoke test as explicit entry points.

The first milestone requires a held-out positive clip with one inspection proposal, a healthy clip with zero proposals, and an ambiguous clip with an explicit insufficient-evidence result. A reviewer can open the stored frames for each reviewed observation. The simulation milestone additionally requires an updated observation with a newly captured closer view.

## Task 0 — Prepare the branch

- [ ] Inspect `git status --short --branch`, confirm the base SHA, and check for an existing harness branch. If the branch already exists, inspect its ancestry and use it if it is the intended branch.
- [ ] Use Superpowers’ using-git-worktrees skill to create an isolated worktree on `experiment/agri-ai-harness` from the verified base. A worktree shares the same Git repository and gives the harness its own checkout.
- [ ] Confirm `git branch --show-current` returns `experiment/agri-ai-harness`. Copy this plan to `docs/superpowers/plans/2026-11-24-ringquad-ai-harness.md` in that worktree and record the actual base if it has moved.
- [ ] Run the existing ROS controller tests in the project’s Podman environment and record the starting result for comparison in Task 5.

## Task 1 — Replay with a reproducible session bundle

**Files:** `harness/pyproject.toml`, `harness/uv.lock`, `harness/.gitignore`, `harness/src/ringquad_harness/{schemas,video,session,cli}.py`, `harness/tests/test_replay.py`.

**Interfaces:** `iter_video(path: Path, session_id: str, camera_id: str) -> Iterator[Frame]`; `SessionWriter.write_event(event: SessionEvent) -> None`; CLI `ringquad-harness replay VIDEO --output DIR --stage ingest`.

- [ ] Write tests that assert a fixture with timestamps at 0, 40, and 115 ms produces those capture times in order, and a skipped frame leaves a sequence gap with the next frame’s original timestamp. Assert that invalid video exits nonzero and has no completed session manifest.
- [ ] Run `uv run --project harness pytest harness/tests/test_replay.py -q`. Confirm failure on the missing replay behavior.
- [ ] Decode with PyAV using frame presentation timestamps and time base. Normalize to the first valid presentation time. Fail explicitly if timestamps needed by replay are missing or invalid. Support paced playback and unpaced evaluation using the same capture timestamps.
- [ ] Write `events.jsonl` and a final `manifest.json` with configuration and stream statistics. Record decoded, processed, skipped, and dropped counts, distinguishing deliberate sampling from backpressure. Use a fresh output directory per run so reruns leave prior evidence available.
- [ ] Run the replay tests and CLI on one local clip. Confirm the manifest and event timestamps agree. Commit as `feat: add video replay and session recording`.

## Task 2 — Track one visible anomaly and save its evidence

**Files:** `harness/src/ringquad_harness/{perception,tracking,pipeline}.py`, `harness/tests/test_tracking.py`, `harness/eval/{manifest.json,README.md}`. Extend `session.py` and `cli.py`.

**Interfaces:** `detect_candidates(frame: Frame) -> list[Candidate]`; `Tracker.update(frame: Frame, candidates: Sequence[Candidate]) -> list[Observation]` emits newly reviewable observations. CLI stage `perception`.

- [ ] Label positive, healthy, and ambiguous crop clips by time range and rough region. Split by recording session so adjacent portions of one recording cannot appear in both tuning and evaluation. Document clip provenance and retrieval.
- [ ] Use synthetic frame tests for a candidate present across three sampled views, a one-frame glint, and a one-view occlusion. Assert one stable observation for the persistent candidate, zero for the glint, and zero duplicate reviews after the brief occlusion.
- [ ] Run `uv run --project harness pytest harness/tests/test_tracking.py -q`. Confirm the tests fail before tracker implementation.
- [ ] Implement a configurable color-and-region baseline in OpenCV. Account for camera motion when associating regions across frames. Track against capture timestamps, allow brief occlusions, and emit a review once per track. Make the three-view fixture threshold explicit in the test configuration.
- [ ] Store a context view and sharp crop keyed by evidence frame ID. Verify the clip evaluation reports matched observations, false alerts, and misses. Put false alerts from normal color changes or motion in the eval notes.
- [ ] Run replay and tracking tests, then run the checked-in fixture manifest against locally available clips. Commit as `feat: track canopy anomalies and retain evidence`.

## Task 3 — Add local reasoning and verify the first milestone

**Files:** `harness/src/ringquad_harness/{agent,model_mlx,evaluate}.py`, `harness/tests/test_agent.py`, `harness/README.md`. Extend `schemas.py`, `pipeline.py`, and `cli.py`.

**Interfaces:** `ReviewBackend.review(observation: Observation, evidence: Sequence[EvidenceFrame]) -> Decision` is asynchronous. `validate_decision(decision: Decision, observation: Observation) -> Decision` enforces valid evidence references. CLI stage `review` and command `evaluate MANIFEST --output DIR`.

- [ ] Write offline tests for a valid inspect decision, cross-track evidence rejection, timeout, and invalid output. Assert that timeout and invalid output produce a recorded review failure and zero inspection proposals.
- [ ] Run `uv run --project harness pytest harness/tests/test_agent.py -q`. Confirm the missing behavior causes failure.
- [ ] Feed the selected images and observation metadata through Pydantic AI. Use the local model’s supported output mode and validate decisions independently of the schema. Require observed frames and a requested view for an inspection. Use a bounded review queue, timeout, and per-track deduplication.
- [ ] Smoke-test small model candidates on the M4 using the same wide views and crops. Check support for multiple images and the chosen structured output mode before pinning a model revision. Compare evidence quality, abstention, review latency, and peak memory. Record which checkpoint and settings won and why. Keep the first model call self-contained with a fixed evidence bundle. Add read-only lookup tools when the evaluation shows a need.
- [ ] Run `uv run --project harness pytest harness/tests -q`, then run the evaluation CLI on held-out clips with the pinned model. Verify one evidence-backed proposal on the positive clip, zero on healthy footage, and explicit insufficient evidence on the ambiguous clip. Store the results, evidence paths, and run command. Record review latency, throughput, dropped frames, and peak memory before setting performance budgets.
- [ ] Commit as `feat: add local scouting decisions and evaluation`.

## Task 4 — Stream simulated video and telemetry into the harness

**Files:** `src/ringquad_description/urdf/ringquad.urdf.xacro`, `src/ringquad_description/launch/gazebo.launch.py`, `src/ringquad_description/config/camera_bridge.yaml`, `src/ringquad_control/launch/demo.launch.py`, `src/ringquad_inspection_bridge/` package, `harness/src/ringquad_harness/live.py`, `harness/tests/test_live.py`. Add container dependencies only where needed.

**Interfaces:** `PoseBuffer.match(frame: Frame, max_age_ns: int) -> PoseSample | None`. Live CLI `ringquad-harness live --url URL --output DIR`. For the local simulator-to-host connection, use HTTP over a Podman port bound to loopback with records consisting of a four-byte big-endian JSON length, a four-byte big-endian JPEG length, then the two payloads. Give fixture replay and live input the same `Frame` and observation contracts.

- [ ] Write tests asserting that an exact-time sample matches, an old or different-clock sample is dropped, and missing pose still yields an observation with no world target. Use simulated nanosecond timestamps in the tests.
- [ ] Run `uv run --project harness pytest harness/tests/test_live.py -q`. Confirm the missing live behavior causes failure.
- [ ] Add an opt-in camera and launch configuration. Bridge image and calibration topics. Subscribe to those topics, `/odom`, `/ringquad/state`, and `/ringquad/lock_fault` in the bridge. Preserve capture timestamps and calibration identity in the handoff. Reset queued samples on clock reset. Use a bounded output queue with drop counts when the host is slow.
- [ ] Build the ROS workspace with `colcon build --symlink-install` inside Podman. Launch with `ros2 launch ringquad_control demo.launch.py mission:=false camera:=true`. Verify camera images and calibration arrive, and a session on the M4 writes frames joined to simulation-time odometry.
- [ ] Pause the stream and verify reconnect and stale-pose behavior. Commit as `feat: connect Gazebo camera and telemetry to the harness`.

## Task 5 — Close one inspection loop in simulation

**Files:** `src/ringquad_inspection_bridge/ringquad_inspection_bridge/{inspection,targeting}.py`, `src/ringquad_inspection_bridge/test/test_inspection.py`, `src/ringquad_description/worlds/scouting.sdf`, `scripts/scouting_sim_test.py`. Extend the bridge, host CLI, and demo launch from Task 4. Add a `world` argument to the demo launch with its existing world as the default.

**Interfaces:** `resolve_target(observation: Observation, pose: PoseSample, camera: CameraCalibration, plane: GroundPlane) -> TargetPose | None`; `InspectionController.submit(request: InspectionRequest) -> InspectionStatus`. The local bridge exposes `POST /inspection-requests` and `GET /inspection-requests/{request_id}`.

- [ ] Test a known ray/plane intersection and rejection for an unresolved or out-of-bounds target. With a fake controller, test stale pose, fault, wrong mode, duplicate ID, and feedback timeout. Assert zero publishes for a rejected request, no second command sequence for a duplicate ID, and a terminal failed status after lost feedback.
- [ ] Run these bridge tests in Podman with `colcon test --packages-select ringquad_inspection_bridge` and `colcon test-result --verbose`. Confirm failures before implementing the behaviors.
- [ ] Use a flat-ground scouting scene with a high-contrast patch. Resolve a target using calibration and the camera-to-body transform. Use a known plane for this simulation slice. Validate the target against the scene’s bounds and enforce pose freshness, controller mode, and fault checks before publishing.
- [ ] Reuse the mode confirmation, setpoint ramping, and odometry completion behavior in `src/ringquad_control/ringquad_control/mission.py` as the reference. Launch `ros2 launch ringquad_control demo.launch.py mission:=false camera:=true world:=scouting.sdf`. Keep a single active request, cease setpoint progression on feedback timeout, and record the failure. The adapter is the sole mission-level publisher. Close the observation only after the harness receives a fresh closer view and links it to the original ID.
- [ ] Run bridge tests, the existing controller tests, and `scripts/scouting_sim_test.py` against the running Podman and M4 pair. Save one successful trace through accepted, running, completed, and new evidence, plus traces for stale telemetry and interrupted feedback. Run the existing demo mission with its default launch settings as a regression check.
- [ ] Commit as `feat: complete simulated agricultural inspection loop`.

## Handoff evidence

The branch should contain a reproducible replay command, fixture manifest, model settings, and an example session report with clickable evidence paths. The simulation report should record the exact base and harness SHAs, launch command, ROS test summary, request status trace, and follow-up frame IDs. Hardware work can use the same contracts once camera transport, time synchronization, and field localization are established.
