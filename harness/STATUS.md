# Local scouting AI: status report

Date: 2026-11-24. Branch: `experiment/agri-ai-harness`. Everything below runs on one laptop, with no cloud calls.

## What exists

Video goes in, and a decision comes out: `inspect`, `monitor` or `insufficient_evidence`, with evidence frame ids.

1. **Detector.** Colour-based candidate proposals (dark lesion-like regions on green foliage). Size limits scale with frame area. A candidate needs green vegetation around it, so dark soil on bare soil is dropped.
2. **Tracker.** Follows a candidate across frames with motion compensation. It needs at least 3 views before it reports one.
3. **Review model.** Qwen3-VL-2B-Instruct (4-bit, MLX, pinned revision `9c4f5209`) looks at the evidence crops and decides.
4. **Live path.** The simulator camera and pose stream into the harness. Each frame is joined to the newest earlier pose.
5. **Closed loop.** An `inspect` decision becomes a request to the simulated RingQuad. It flies to the spot and the harness collects a follow-up frame. Requests with stale or missing pose are rejected.

## Results

**Tests.** 41 harness tests pass, plus unit tests for the camera bridge, ground targeting and the inspection controller.

**Simulation, end to end.** The demo mission, camera streaming with pause and reconnect, and the inspection loop in three scenarios (success, stale pose, interrupted) all ran with the real Qwen model.

**Real video, perception** (every 5th frame, 1280x720):

| Clip | Expected | Before detector fix | After |
|---|---|---|---|
| Wheat, healthy | 0 | 19 | 0 |
| Paddy aerial, dry patch only | few | 78 | 7 |
| Corn close-ups | few | 1,081 | 280 |
| Vlog | few | 735 | 245 |

The before and after counts use different counting methods (events vs distinct tracks), so the size of the drop is approximate.

**Real video, model review** (31 tracks):

| Clip | Reviewed | inspect | monitor | insufficient_evidence | failed |
|---|---|---|---|---|---|
| Paddy | 7 | 6 | 1 | 0 | 0 |
| Corn close-ups | 12 of 280 | 0 | 6 | 3 | 3 |
| Vlog | 12 of 245 | 1 | 5 | 2 | 4 |

On paddy, the `inspect` explanations describe dry yellow-brown vegetation, which matches a real dry patch in the footage. Healthy corn got no `inspect` calls. The one `inspect` in the vlog was a false positive on grass.

## Limits

- **No accuracy number.** There is no labelled ground truth, so there is no precision or recall. The above is manual inspection of a handful of clips.
- **One real aerial clip**, and its patch is dry vegetation, not a confirmed disease. No real diseased-crop aerial footage has been tested.
- **Review failures.** About 1 in 4 model reviews fail with "Exceeded maximum output retries (0)" (5 of 13 in simulation, 7 of 31 on real video). They are recorded as failures with no proposal. The cause is not fixed.
- **Close-ups stay noisy.** Soil clods between leaves still pass the colour and context checks. Only 12 tracks per clip were reviewed.
- **Not checked:** whether the paddy boxes sit on the patch itself, and the unreviewed tracks.
- **Lost odometry** is unit-tested only, not run in simulation.
- **Specialised models not adopted.** Grounding DINO tiny was tried as a text-prompt scout on 8 frames. It returned near-whole-frame boxes, including 0.47 on healthy corn, so it is not part of the system. SAM 2.1, the disease classifiers and the retrieval layer were not tested.

## Notes on the test footage

The real clips are third-party YouTube videos used for internal testing only. They are not in this repository, and commercial use needs licences from the owners.

| Clip | Source |
|---|---|
| Wheat, low flight | [WXnFXSjcmWE](https://www.youtube.com/watch?v=WXnFXSjcmWE) "aerial drone close up low flight above ripe yellow wheat field" |
| Paddy flight | [mGE_94bP_Vg](https://www.youtube.com/watch?v=mGE_94bP_Vg) "Drone Flight over Crops from a DJI PHANTOM" |
| Corn nutrient deficiency | [1bQMEkWDsQs](https://www.youtube.com/watch?v=1bQMEkWDsQs) "Nutrient Def In Corn #636", channel Ag PhD |
| Vlog | [vNN8RLy1qBo](https://www.youtube.com/watch?v=vNN8RLy1qBo) "Drone Scouting - Crop Update / Nitrogen Deficient Corn", channel iowANFarmer |

None of these is reuse-licensed. Credit the channels when showing frames, and obtain a licence before any commercial use. One clip carries a burned-in caption naming the deficiency, which could leak the answer to the model on those frames.

## Next

1. Raise the output retry limit and re-run the failed reviews.
2. Check paddy box locations against the patch.
3. Collect or label aerial footage of a real diseased patch, so accuracy can be measured.
