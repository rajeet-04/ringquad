"""Run photo-derived perception regressions and report match/false-alert/miss counts."""
import argparse
import json
from pathlib import Path

from ringquad_harness.cli import replay


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("manifest.json"))
    parser.add_argument("--clips", type=Path, required=True, help="directory containing the MP4 paths listed in the manifest")
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
        region = clip.get("region_xyxy_approx")
        time_start, time_end = clip["time_range_ns"]
        remaining = set(range(len(observations)))
        matched = 0
        if expected and region:
            rx1, ry1, rx2, ry2 = region
            for _ in range(expected):
                eligible = []
                for index in remaining:
                    observation = observations[index]
                    first, last = observation["first_capture_ns"], observation["last_capture_ns"]
                    x1, y1, x2, y2 = observation["bbox"]
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    time_match = first <= time_end and last >= time_start
                    region_match = rx1 <= cx <= rx2 and ry1 <= cy <= ry2
                    if time_match and region_match:
                        eligible.append(index)
                if not eligible:
                    break
                chosen = eligible[0]
                remaining.remove(chosen)
                matched += 1
        misses = max(0, expected - matched)
        false_alerts = len(remaining)
        tp += matched; fp += false_alerts; fn += misses
        rows.append({"clip_id": clip["clip_id"], "label": clip["label"],
                     "expected": expected, "observations": len(observations),
                     "matched": matched, "false_alerts": false_alerts, "misses": misses,
                     "review_outcome_target_only": clip.get("expected_review_outcome"),
                     "observation_ids": [o["observation_id"] for o in observations],
                     "session": clip["clip_id"]})
    report = {"provenance_type": manifest["provenance_type"], "counts": {"matched_observations": tp,
              "false_alerts": fp, "misses": fn}, "clips": rows,
              "notes": "Photo-derived fixture results are not field accuracy. A detection matches only when its track time interval overlaps the labeled range and its reported bbox center falls inside the approximate labeled region. Review outcome values are targets only; this perception runner does not evaluate Task 3 model decisions. False alerts can arise from normal shadows, leaf markings, soil/background, or camera motion."}
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
