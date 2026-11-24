# Perception regression fixtures

These are **photo-derived regression fixtures, not field video** and cannot establish field accuracy. `manifest.json` records their labels, source session IDs, approximate image-space crop ranges, generated time bounds, expectations, and source SHA-256 values. Each clip comes from a separate photograph. These fixtures were used during development and model selection. The separate synthetic tests exercise persistence, camera translation, glints, and occlusion deterministically. The renderer keeps scale fixed and creates camera motion through image translation only.

The photographs are by Dwight Sipler, licensed CC BY 2.0. Attribution: “Photos by Dwight Sipler, used under CC BY 2.0”; source titles and original filenames are listed in the manifest and generator. Retrieval paths and expected checksums are kept in `prepare_photo_clips.py`. To retrieve originals and regenerate clips:

```sh
uv run --project harness python harness/eval/prepare_photo_clips.py \
  --source-dir ../photo-sources --output ../photo-clips
```

The generated MP4s and original photographs stay outside Git. The current environment has generated clips at `../photo-clips/{positive,healthy,ambiguous}.mp4`. Their crops are approximate labels: positive `[158,122,252,207]`, ambiguous `[389,190,437,243]`, healthy has no anomaly region.

Run the manifest evaluator with an explicit clip directory and a fresh output directory:

```sh
PYTHONPATH=harness/src python harness/eval/run_manifest.py \
  --clips ../photo-clips --output /tmp/ringquad-photo-eval
```

The evaluator checks time overlap and whether each reported bbox center is inside the approximate labeled crop. Unmatched detections are false alerts and unmatched expected regions are misses. `expected_review_outcome` records a Task 3 target only; this perception-only evaluator does not check it.

The OpenCV baseline uses low-value, low-saturation regions and temporal association. Expected false-alert sources include shadows/lighting that darken healthy leaves, soil/background entering the frame, global camera motion causing phase-correlation alignment errors, and lesion-like veins or normal leaf markings. A proposal is not a disease diagnosis.
