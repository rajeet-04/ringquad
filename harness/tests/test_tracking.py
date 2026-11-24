from pathlib import Path

import numpy as np

from ringquad_harness.schemas import Candidate, Frame
from ringquad_harness.tracking import Tracker, TrackerConfig


def frame(index: int, image: np.ndarray | None = None) -> Frame:
    if image is None:
        image = np.full((120, 160, 3), (45, 130, 55), dtype=np.uint8)
        # Stable textured background supports camera motion estimation.
        for y in range(0, 120, 12):
            image[y:y + 2, :, :] = (40 + y % 30, 115, 50)
    return Frame("fixture-session", "camera-0", index, index * 125_000_000,
                 "synthetic", image)


def candidate(x: int = 48) -> Candidate:
    return Candidate((x, 42, x + 20, 62), 0.9, 0.8)


def test_persistent_candidate_survives_camera_translation_and_emits_once(tmp_path: Path) -> None:
    tracker = Tracker(TrackerConfig(min_views=3, evidence_dir=tmp_path))
    emitted = []
    # Translation mimics a small camera pan; candidate location follows it.
    for i in range(5):
        img = np.roll(frame(i).image, i * 2, axis=1)
        emitted.extend(tracker.update(frame(i, img), [candidate(48 + i * 2)]))
    assert len(emitted) == 1
    obs = emitted[0]
    assert obs.first_capture_ns == 0 and obs.last_capture_ns >= 250_000_000
    assert len(obs.evidence_frame_ids) >= 3
    assert obs.context_path and Path(obs.context_path).exists()
    assert obs.crop_path and Path(obs.crop_path).exists()
    # Continue observing the same patch; one review per track.
    for i in range(5, 8):
        emitted.extend(tracker.update(frame(i), [candidate()]))
    assert len(emitted) == 1


def test_one_frame_glint_never_becomes_reviewable() -> None:
    tracker = Tracker(TrackerConfig(min_views=3))
    assert tracker.update(frame(0), [candidate()]) == []
    assert tracker.update(frame(1), []) == []
    assert tracker.update(frame(2), []) == []


def test_single_view_occlusion_does_not_duplicate_review() -> None:
    tracker = Tracker(TrackerConfig(min_views=3))
    emitted = []
    for i, present in enumerate([True, True, True, False, True, True]):
        emitted.extend(tracker.update(frame(i), [candidate()] if present else []))
    assert len(emitted) == 1
    assert len({obs.observation_id for obs in emitted}) == 1


def test_two_tracks_have_distinct_bounded_evidence(tmp_path: Path) -> None:
    tracker = Tracker(TrackerConfig(min_views=3, evidence_dir=tmp_path))
    emitted = []
    for i in range(5):
        emitted.extend(tracker.update(frame(i), [candidate(25), candidate(105)]))
    assert len(emitted) == 2
    assert len({obs.observation_id for obs in emitted}) == 2
    all_paths = [path for obs in emitted for item in obs.evidence
                 for path in (item.context_path, item.crop_path)]
    assert len(all_paths) <= 12  # at most three frame pairs for each of two tracks
    assert len(all_paths) == len(set(all_paths))
    assert all(Path(path).exists() for path in all_paths)
    for obs in emitted:
        assert len(obs.evidence) <= 3
        assert tuple(item.frame_id for item in obs.evidence) == obs.evidence_frame_ids
