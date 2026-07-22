# Clustering evaluation vs. ground truth — offline .pcd input (2026-07-13 bags)

## What changed from the previous run

The earlier evaluation fed the detector via `environment_ros` subscribed to a **live rosbag replay**
(`rosbag play --clock`). That path lost frames: ROS message drops, `--clock`/sim-time sync, and
nearest-timestamp frame pairing meant the predicted-frame count didn't always match the ground-truth
frame count 1:1, so true positives came out lower than the number of labeled frames even when the
detector was actually finding the person in nearly every frame.

This run uses a new offline batch tool, `environment_offline_eval` (`src/environment_offline_eval.cpp`,
built as part of the existing CMake project), that calls `detectObstacles()` directly on each `.pcd`
file already saved on disk — the same frames the ground truth was labeled from — with **no ROS, no
rosbag replay, no timestamp resync**. Each output JSON is named after its input frame's filename, so
frame correspondence with ground truth is exact and lossless:

```
./build/environment_offline_eval <scene>/lidar <output_dir>
```

## How to read the metrics

`scripts/evaluate_detections.py` supports two matching metrics (`--metric`), both restricting boxes to
`--range` meters of the sensor origin first:

- **`center`** (default): a pair counts as a match if the distance between box centers is within
  `--match-dist` (0.5 m). Simple and robust, but ignores box size/shape entirely — a tiny box
  dead-center on a huge one scores identically to two well-fit boxes of matching size.
- **`iou`**: standard object-detection 3D intersection-over-union matching, the metric more commonly
  used for this kind of benchmark. A pair counts as a match if IoU ≥ `--iou-thresh` (0.5 here — the
  conventional PASCAL-VOC/KITTI-style bar for a "correct" detection). Predicted boxes are always axis-aligned (`BoundingBox()` only produces
  axis-aligned boxes) while GT boxes carry a real yaw, so IoU is computed between the rotated GT
  rectangle and the axis-aligned predicted rectangle in the ground plane via polygon clipping
  (Sutherland–Hodgman), multiplied by the overlap of their z-extents, divided by union volume.
  Sanity-checked against known closed-form cases (identical boxes → 1.0, disjoint → 0.0, 50%-overlap-in-x
  unit cubes → 0.333, two same-center unit squares 45°-apart → 0.707, matching the analytical
  2√2−2 intersection-area result) before use.
- **`containment`**: existence-only matching, purpose-built for a **near-range stop-area / safety
  trigger** use case — where the question isn't "how precisely does the box match the person's shape"
  but simply "did the system notice someone was there, and did it false-alarm." A GT person counts as
  detected if their center point falls inside a predicted box, expanded by `--containment-margin`
  (0.15 m here) per side to tolerate boundary slop. Box size/shape mismatch is deliberately not
  penalized — irrelevant to whether a stop should trigger. Reports **Detection Rate** (= recall — the
  safety headline, want ≈1.0) and **False Alarm Rate** (FP per frame, not FP/(TP+FP) — a stop-area
  system cares how often it fires when it shouldn't, independent of how many real detections exist,
  which is what precision would conflate it with) instead of precision/F1/IoU.
  Sanity-checked directly (center point → contained; just-outside point → not contained without
  margin, contained with margin; correctness verified under 90°-rotated boxes too) before use.

`center` and `iou` share:
- **True positive (TP)**: a predicted box matched to a real ground-truth box — the detector correctly
  found the person.
- **False positive (FP)**: a predicted box with no matching ground-truth box — the detector reported
  something (e.g. clutter, a wall/floor edge) that wasn't the labeled person.
- **False negative (FN)**: a ground-truth person with no matching predicted box — the detector missed
  them entirely in that frame.
- **Precision = TP / (TP + FP)**, **Recall = TP / (TP + FN)**, **F1 = 2·P·R / (P + R)** — standard
  definitions; F1 punishes whichever of precision/recall is worse.
- **Mean matched score** — for matched pairs only, the average center error (m, `center` metric) or
  average IoU (`iou` metric): how good the correct detections are, not just whether they exist.

## Results — center-distance metric (match ≤0.2m)

| Scene | Frames | TP | FP | FN | Precision | Recall | F1 | Mean center err (m) |
|---|---|---|---|---|---|---|---|---|
| `20260713Concrete2m-1m_range_2026-07-13-16-51-00` (range ≤2.5m) | 435 | 242 | 49 | 6 | 0.832 | 0.976 | 0.898 | 0.097 |
| `20260713Concrete2m_2026-07-13-16-48-16` (range ≤2.5m) | 304 | 144 | 164 | 160 | 0.468 | 0.474 | 0.471 | 0.131 |
| `20260713Concrete2m_range_2026-07-13-16-49-41` (range ≤2.5m) | 531 | 16 | 97 | 89 | 0.142 | 0.152 | 0.147 | 0.156 |
| `20260713Concrete2m_range_2026-07-13-16-49-41` (range ≤10m, full clip) | 531 | 178 | 366 | 353 | 0.327 | 0.335 | 0.331 | 0.145 |

## Results — 3D IoU metric (match ≥0.5 IoU)

| Scene | Frames | TP | FP | FN | Precision | Recall | F1 | Mean matched IoU |
|---|---|---|---|---|---|---|---|---|
| `20260713Concrete2m-1m_range_2026-07-13-16-51-00` (range ≤2.5m) | 435 | 147 | 144 | 101 | 0.505 | 0.593 | 0.545 | 0.587 |
| `20260713Concrete2m_2026-07-13-16-48-16` (range ≤2.5m) | 304 | 38 | 270 | 266 | 0.123 | 0.125 | 0.124 | 0.583 |
| `20260713Concrete2m_range_2026-07-13-16-49-41` (range ≤2.5m) | 531 | 0 | 113 | 105 | 0.000 | 0.000 | nan | nan |
| `20260713Concrete2m_range_2026-07-13-16-49-41` (range ≤10m, full clip) | 531 | 134 | 410 | 397 | 0.246 | 0.252 | 0.249 | 0.566 |

## Results — existence/containment metric (margin 0.15m) — the stop-area-relevant numbers

| Scene | Frames | TP | FP | FN | Detection Rate | False Alarm Rate | Frames w/ FA |
|---|---|---|---|---|---|---|---|
| `20260713Concrete2m-1m_range_2026-07-13-16-51-00` (range ≤2.5m) | 435 | 248 | 43 | 0 | 1.000 | 0.099/frame | 9.2% |
| `20260713Concrete2m_2026-07-13-16-48-16` (range ≤2.5m) | 304 | 304 | 4 | 0 | 1.000 | 0.013/frame | 1.3% |
| `20260713Concrete2m_range_2026-07-13-16-49-41` (range ≤2.5m) | 531 | 72 | 41 | 33 | **0.686** | 0.077/frame | 7.7% |
| `20260713Concrete2m_range_2026-07-13-16-49-41` (range ≤10m, full clip) | 531 | 462 | 82 | 69 | 0.870 | 0.154/frame | 14.7% |

All predicted frames matched 1:1 with ground-truth frames in every scene (435/435, 304/304, 531/531) —
confirming the earlier TP-vs-frame-count discrepancy was a rosbag-replay artifact, not a detector
issue.

Scene 3 (`2m_range_...16-49-41`, the "随意走动" free-walking test) is reported at two range cutoffs
because the person walks out to ~6m from the sensor for most of the clip, well past the 2.5m cutoff
used for the other two (static, close-range) scenes — the ≤2.5m row only scores the fraction of frames
where the person happens to be nearby, while the ≤10m row scores the whole 53.5s walk.

## Interpretation

**Both thresholds were tightened from the earlier pass** (center match 0.5m → 0.2m; IoU match 0.3 →
0.5, the conventional detection-benchmark bar) at the user's request, and the picture changes sharply:
what looked like a strong pipeline under the looser thresholds (center-F1 0.92–0.99, IoU-F1 0.27–0.90)
now reads as weak across the board (center-F1 0.15–0.90, IoU-F1 0–0.55). Scene `2m_2026-07-13-16-48-16`
in particular collapses from the best-scoring scene (0.99 center-F1 @0.5m) to one of the worst (0.47
center-F1 @0.2m) — meaning most of its "correct" detections were only within half a meter of the
person, not tightly on top of them. Scene 3 at ≤2.5m fails outright under IoU≥0.5 (0 matches at all —
every predicted box misses every ground-truth box by more than 50% of their union volume in that
near-field slice).

This is a real signal, not a metric artifact: `BoundingBox()` fits an axis-aligned box to whatever
points survive clustering, and those boxes are frequently smaller than, or offset from, the true
person extent — a gap that only shows up once the match tolerance is tightened enough to demand real
overlap rather than "somewhere in the vicinity." Mean matched IoU (0.57–0.59, when there are matches
at all) is well above the 0.5 pass bar for the pairs that *do* pass, meaning the failures are mostly
outright misses (no box within tolerance), not marginal near-misses — consistent with the
wall/floor-edge fragmentation artifact noted in the repo's `experiments/2026-07-21_close_range_tuning.md`.
At these stricter thresholds, this pipeline should not be considered production-ready for precise
box-shape use cases; `BoundingBox()`/cluster-completeness at all ranges (not just 3-6m) is the
priority fix.

### For the actual use case (near-range stop-area trigger): use the containment numbers, not IoU/center

The center-distance and IoU tables above are the wrong lens for this system — they penalize a stop
trigger for reporting a box that's a different *shape* than the hand-labeled ground truth, which is
irrelevant to whether the stop should fire. The **containment** table is the one that answers the
actual operational questions:

- **Detection Rate is what matters most, and it's a mixed picture.** The two static/close scenes
  (`2m-1m_range`, `2m_wall`) hit a perfect 1.000 — the system never misses the person when they're
  near the sensor and roughly stationary. But scene 3, the free-walking test, drops to **0.686** at
  ≤2.5m and only reaches **0.870** even over the full 10m walk — meaning **roughly 1 in 3 near-range
  frames, and 1 in 8 frames overall, would be a missed stop** if this pipeline drove a safety trigger.
  That's the number that should gate any go/no-go decision, not the IoU or center-distance scores.
- **False Alarm Rate is low but non-zero everywhere** (0.01–0.15 spurious detections per frame, 1.3–14.7%
  of frames with at least one false alarm) — an occasional unnecessary stop, which is a nuisance/cost
  problem, not a safety problem, and clearly the secondary concern relative to missed detections here.
- Scene 3's much worse Detection Rate (vs. the two static scenes' perfect score) says the failure mode
  is range/distance-dependent, not a fundamental inability to see a person — worth checking whether it
  degrades gradually with distance or whether there's a specific range band where the detector loses
  the person outright, before concluding the pipeline needs a rewrite vs. a re-tune of `EuclideanClustering`'s
  `minClusterSize`/`clusterTolerance` for sparser far-range returns.

## Scene 3 ground truth: fixed

`20260713Concrete2m_range_2026-07-13-16-49-41`'s ground truth was originally produced by an automated
offline box-fitter that locked onto the static wall instead of the walking person (see earlier
investigation: a persistent, unmoving 1466-point cluster at x≈−1.3 to −1.6m vs. a genuine ~380-point
person-sized cluster nearer x≈+2m). It has since been re-labeled by hand in the SUSTechPOINTS tool,
using the auto-tracker in ≤40-frame batches with an in-memory position check after every `Auto` call
(rejecting/redoing any batch where the tracked x-position jumped >1.5m between adjacent frames, which
is exactly the wall-drift failure mode) before finalizing. All 531 frames are labeled; a post-hoc
point-cloud check confirms every box contains ≥5 real lidar points (none empty/spurious) and no
frame-to-frame position jump exceeds 1.5m anywhere in the sequence.

## Artifacts

Per-frame prediction JSON for all 3 bags: `results/eval_run_offline/{2m1m_range_1651,2m_1648,2m_range_1649}/`
