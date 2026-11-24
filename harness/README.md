# RingQuad local scouting harness

The harness replays video with original capture timestamps, detects and tracks a candidate canopy color change, stores context/crop evidence, then optionally asks a local vision model for a review decision. `inspect` is a request for a closer image, not a disease diagnosis. A reviewer can open the saved image paths in each observation event. Recorded two-pass replay (perception bundle, then model review) is covered; ROS camera integration remains future scope.

## Install

Use the optional MLX extra on Apple Silicon macOS for local vision inference:

```sh
uv sync --project harness --extra mlx --group dev
```

Model weights are not downloaded by the harness. Supply an already-present MLX-VLM model directory with `--model-path`. The evaluation fixtures and model weights live outside Git. The demonstrated checkpoint is `mlx-community/Qwen3-VL-2B-Instruct-4bit`, revision `9c4f5209e57b31f4b9dfba735de3fb983739c9cc`; model paths are intentionally not embedded in project defaults.

## Replay and review

Perception only:

```sh
uv run --project harness ringquad-harness replay VIDEO.mp4 --output /tmp/ringquad-session --stage perception
```

Review the observations from that same run during replay:

```sh
uv run --project harness --extra mlx ringquad-harness replay VIDEO.mp4 --output /tmp/ringquad-reviewed \
  --stage review --model-path /path/to/local/model --review-timeout 60
```

Each review writes a `review` event to `events.jsonl`. Failures include the timeout or invalid output reason and always have zero inspection proposals. The review queue has a bounded capacity and applies backpressure for deterministic replay. Each observation ID is reviewed at most once. MLX runs in a child process that is killed and reaped on timeout/cancellation. The worker has Hugging Face offline flags set, so absent local model assets fail locally rather than triggering downloads.

## Fixture evaluation

The checked-in manifest describes positive, healthy, and ambiguous photo-derived development regression clips. They have been used during model selection, are not held out, and do not establish field accuracy. With the generated fixture clips in `../photo-clips`:

```sh
uv run --project harness --extra mlx ringquad-harness evaluate harness/eval/manifest.json \
  --output ../evaluation-run --media-dir ../photo-clips \
  --model-path /path/to/local/model --review-timeout 60
```

The command writes an `evaluation.json` report plus a replay bundle for every case. It exits `0` only when exact observation counts, expected review outcomes, and proposal counts match; mismatches or replay failures return nonzero. A zero-observation fixture expects no model outcome. The report records frame/drop statistics, capture timing, per-review latency/memory when the MLX runtime exposes it, and bundle-relative evidence paths. Outputs are created fresh so earlier run evidence is preserved.

The current benchmark candidate is the local Qwen3-VL-2B-Instruct 4-bit checkpoint. A preliminary same-view comparison correctly selected `inspect` for the positive and `insufficient_evidence` for the ambiguous sample; the smaller SmolVLM candidate exhausted its generation budget. A full held-out evaluation, aggregate throughput, and peak-memory budget must be taken from the saved run report on the target M4 before treating those observations as a performance guarantee.
