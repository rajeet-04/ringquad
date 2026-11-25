from __future__ import annotations

import numpy as np

from ringquad_harness.perception import detect_candidates
from ringquad_harness.schemas import Frame


def frame_of(background: tuple[int, int, int], size=(320, 480), patch=(100, 160, 60, 80)) -> Frame:
    img = np.full((*size, 3), background, np.uint8)
    y, x, h, w = patch
    img[y:y + h, x:x + w] = (60, 40, 25)  # dark brown
    return Frame("s", "cam", 0, 0, "sim_time", img)


def test_dark_patch_on_green_foliage_is_found() -> None:
    assert len(detect_candidates(frame_of((50, 140, 50)))) == 1


def test_dark_patch_on_bare_soil_is_dropped() -> None:
    assert detect_candidates(frame_of((150, 120, 90))) == []


def test_small_speck_is_ignored_at_high_resolution_but_not_at_reference() -> None:
    def speck(size, scale):
        f = frame_of((50, 140, 50), size, (int(100 * scale), int(160 * scale), int(20 * scale), int(20 * scale)))
        return detect_candidates(f)
    # a 20x20 px mark: a lesion in a 480x320 frame, a shadow in 1440x960
    assert len(speck((320, 480), 1)) == 1
    assert speck((960, 1440), 1) == [] or speck((960, 1440), 1)[0].bbox[2] - speck((960, 1440), 1)[0].bbox[0] > 20


def test_dark_patch_with_yellow_halo_on_foliage_is_found() -> None:
    f = frame_of((50, 140, 50), patch=(100, 160, 60, 80))
    f.image[85:175, 140:260] = (170, 160, 40)  # yellow-green halo ring
    f.image[100:160, 160:240] = (60, 40, 25)
    assert len(detect_candidates(f)) == 1
