"""Run photo-derived perception regressions and report match/false-alert/miss counts."""
import argparse
import json
import shutil
import tempfile
from pathlib import Path

from ringquad_harness.cli import replay


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("manifest.json"))
    parser.add_argument("--clips", type=Path, default=Path(__file__).parents[2] / "photo-clips")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    tp = fp = fn = 0
    for clip in manifest["clips"]:
        session = args.output / clip["clip_id"]
        rc = replay(args.clips / clip["path"], session, "perception")
        if rc:
            return rc
        events = [json.loads(line) for line in (session / "events.jsonl").read_text().splitlines()]
        observations = [event["data"] for event in events if event["event_type"] == "observation"]
        expected = clip["expected_observations"]
        matched = min(expected, len(observations))
        misses = max(0, expected - matched)
        false_alerts = max(0, len(observations) - matched)
        tp += matched; fp += false_alerts; fn += misses
        rows.append({"clip_id": clip["clip_id"], "label": clip["label"],
                     "expected": expected, "observations": len(observations),
                     "matched": matched, "false_alerts": false_alerts, "misses": misses,
                     "review_outcome_target": clip.get("expected_review_outcome"),
                     "observation_ids": [o["observation_id"] for o in observations],
                     "session": clip["clip_id"]})
    report = {"provenance_type": manifest["provenance_type"], "counts": {"matched_observations": tp,
              "false_alerts": fp, "misses": fn}, "clips": rows,
              "notes": "Photo-derived fixture results are not field accuracy. False alerts can arise from normal shadows, leaf markings, soil/background, or camera motion."}
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
