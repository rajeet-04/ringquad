# Slide outline: local scouting AI for RingQuad

About 10 slides, 8 to 10 minutes. Numbers come from [STATUS.md](STATUS.md).

1. **Problem.** Scouting a field for crop stress is slow on foot. A small morphing quadcopter can fly low and land to look closer. We want it to decide where to look, not just record video.
2. **Goal in one line.** Video in, then a decision (`inspect`, `monitor`, `insufficient_evidence`) with evidence, then the drone goes and checks.
3. **System overview.** One diagram: camera, detector, tracker, review model, decision, request to the drone, follow-up frame. Everything runs on a laptop.
4. **Detector and tracker.** Colour-based proposals, green-context check, 3-view tracking. Say plainly that it is a proposal generator, not a disease classifier.
5. **Review model.** Qwen3-VL-2B (4-bit, local). Show one evidence crop next to the model's decision and explanation.
6. **Closed loop demo.** Screen recording of the simulation: patch seen, `inspect`, drone flies there, follow-up image. Mention the stale-pose and interrupted cases were also run.
7. **Real footage: the detector fix.** Wheat 19 false tracks to 0, paddy 78 to 7. Show before and after frames. Credit the video channels on the slide.
8. **Real footage: model review.** Paddy 6 of 7 `inspect` on a real dry patch. Healthy corn gave no `inspect`. One false positive on grass.
9. **What we tried and dropped.** Grounding DINO tiny as a text-prompt scout boxed whole frames and scored 0.47 on healthy corn, so it is out.
10. **Limits and next steps.** No accuracy number yet. About 1 in 4 model reviews fail. One real aerial clip, and its patch is dry vegetation, not a confirmed disease. Next: label 20 to 30 frames for a hit/miss count, fix the review failures, find real diseased-patch footage.

## Say it this way

- "Flagged a discoloured patch for inspection", not "detected disease".
- "Accuracy is not measured yet", said by you before anyone asks.

## Prepare before presenting

- A screen recording of the simulation loop (slide 6).
- Paddy box overlays (slide 8). They are not generated yet.
- Credit lines for the four videos (see the table in STATUS.md).
