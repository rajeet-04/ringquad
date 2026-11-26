# Simulated scouting demo

`success.mp4` is a 17 s silent clip of the full loop in Gazebo (world `scouting`), recorded from the camera bridge stream and
annotated by `harness/eval/render_demo.py`. Frames play at 10 fps; the sim produced about 6 per second, so the clip runs
roughly 1.5x real time.

Left: camera frame, orange boxes are detector candidates, the green box is the confirmed track. Right: stage, Qwen3-VL-2B
decision and explanation, and the inspection request state (accepted, running, completed). The yellow border marks the
follow-up frame linked to the observation.

| file | content |
| --- | --- |
| success.mp4 | the annotated clip |
| success-session.jsonl | harness events for the observation and its review (attempt 4) |
| success-inspection.jsonl | request, state trace, linked follow-up and result |

How it was made (container `ringquad-harness` running `demo.launch.py mission:=false camera:=true world:=scouting gui:=false`):

```bash
uv run --project harness python harness/eval/record_stream.py http://127.0.0.1:8765/stream REC_DIR &
python3 scripts/scouting_sim_test.py --model-path MODEL --out RUN --scenario success --attempts 8
uv run --project harness python harness/eval/render_demo.py REC_DIR RUN/success-session-4 RUN/success-inspection OUT.mp4
```

Read before quoting this as a result:

- The model needed 4 observation sessions to produce an `inspect` decision: `monitor`, `insufficient_evidence`,
  `insufficient_evidence`, then `inspect`. The clip shows the successful one. This is an integration demo, not accuracy.
- The `scouting.sdf` patch was changed before this run. The earlier flat square with three dots got `monitor` in 11
  consecutive attempts, so it is now a ragged blight (rotated yellow-green patches, dark core, scattered lesions).
  The older traces in `../sim-traces/` were recorded on the flat square.
- The same recording exposed a detector bug: the first version of the green-context check rejected lesions with a
  yellow halo, so the model never saw the new patch. Fixed in the commit that precedes this one.
- The drone's own shadow and propellers are visible in the lower left of the frame. That is a sim rendering artefact.
- The sim camera is a plain-colour render, not crop imagery.

## Real-footage clips

`paddy-real.mp4` and `corn-real.mp4` come from `harness/eval/render_real_demo.py`: the detector and tracker run over a window
of a third-party YouTube clip (paddy `mGE_94bP_Vg`, corn `1bQMEkWDsQs`, not in this repo), the first 8 confirmed tracks go
to the local Qwen3-VL-2B model (failed reviews retried up to 2 times), and the window is re-rendered with boxes and the
decisions. The `*-reviews.json` files hold the outcomes and explanations.

- Recorded offline run, not a live result. Outcomes are whatever the model returned; none were filtered.
- Paddy: 4 inspect, 1 monitor, 3 failed. Corn: 2 inspect, 2 monitor, 3 insufficient_evidence, 1 failed.
- Boxes come from the colour detector and are small; on paddy they sit near the dry patch, not around it. The first corn
  `inspect` box sits on a shadow, and its explanation quotes the burned-in caption.
- Reviews are not deterministic: an earlier paddy render gave 7 inspect of 8.
- No accuracy number: there is no labelled ground truth. Footage is credited to its channels and used for internal testing only.
