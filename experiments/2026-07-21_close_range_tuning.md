# Close-range (2m) detection tuning — 2026-07-21

## Goal
Stable detection of a single person standing within ~2m of a stationary
VLP-16 rig, plus a quantitative way to score detection quality and speed.
Starting point was the original highway-driving hyperparameters in
`src/city_block.h`, tested against `/home/firo/Documents/BAG/20260713/*.bag`.

## Problem observed
The person was not consistently detected frame-to-frame in RViz.

## Root cause
`ProcessPointClouds::FilterCloud` (`src/processPointClouds.cpp`) ran the PCL
`VoxelGrid` downsampler on the **raw, full-range** cloud *before* cropping to
the region of interest. `VoxelGrid` sizes its internal voxel index off the
input cloud's full spatial extent, so with the VLP-16's ~100m range and a
fine leaf size, the index overflowed PCL's internal 32-bit indexing:

```
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
```

When this triggers, voxelization silently degrades for that frame, so point
density (and everything downstream: plane segmentation, clustering) becomes
inconsistent frame-to-frame — this was the direct cause of the intermittent
detection.

## Fix
Reordered `FilterCloud` to crop to the ROI box first, then voxel-grid the
now-small cropped cloud. This bounds the voxel grid's extent to
`minPoint`/`maxPoint` regardless of `filterRes`, eliminating the overflow.
As a side effect it's also faster, since voxelizing fewer points is cheaper.

## Hyperparameter changes (`src/city_block.h`, `detectObstacles()`)

| param              | before (highway) | after (2m close-range) | why |
|---------------------|------------------|------------------------|-----|
| `filterRes`         | 0.4 m            | 0.05 m                 | a person at 2m is a small, sparse cluster; 0.4m voxels were collapsing/losing it |
| `minPoint`/`maxPoint`| (-10,-6.5,-2) to (30,6.5,1) | (-2.5,-2.5,-1) to (2.5,2.5,1.5) | tight crop around the sensor instead of a car's forward road view |
| `distanceThreshold` | 0.3 m            | 0.08 m                 | finer plane-fit tolerance for an indoor concrete floor at close range |
| `clusterTolerance`  | 0.5 m            | 0.3 m                  | avoid merging separate nearby objects at person body-scale |
| `minClusterSize`    | 10               | 5                      | tolerate sparser frames without dropping the detection |
| `maxClusterSize`    | 140              | 2000                   | finer voxel leaf means a person cluster now has far more points than before |
| `egoRadius` (new)   | n/a              | 0.5 m                  | drop points within 0.5m of the sensor origin — tripod mount / near-field returns, see below |

## Ego-point removal (new parameter)
Added `egoRadius` to `ProcessPointClouds::FilterCloud`
(`src/processPointClouds.h`/`.cpp`), default `0.0f` (disabled, so the
offline `.pcd` highway path is unaffected unless it opts in). When > 0, any
point with `x²+y²+z² < egoRadius²` (distance from sensor origin) is dropped
after the existing crop-box/roof-box filtering. Wired up in
`city_block.h`'s `detectObstacles()` as `egoRadius = 0.5` (meters).

This is separate from the pre-existing hardcoded "roof" `CropBox`
(`x∈[-1.5,2.6], y∈[-1.7,1.7], z∈[-1,-0.4]`), which models a car's roof
returns and doesn't match a tripod-mounted static rig's geometry — that box
is left in place but is essentially a no-op for this geometry.

## Current ROI, end to end (see "How the ROI is set up" section in the
follow-up message for the full explanation)
- Crop box: 5m x 5m x 2.5m volume centered on the sensor (`minPoint`/`maxPoint` above).
- Ego exclusion: sphere of radius 0.5m around the sensor origin.
- Ground plane vs. obstacles: RANSAC-segmented within that cropped volume.
- Clustering: Euclidean/KD-tree clustering within the obstacle points.

## Detection-speed instrumentation
Added per-frame timing (`std::chrono`) in `environment_ros.cpp`'s
`cloudCallback`, logging a running average every 30 frames via
`ROS_INFO_STREAM`. No change to the offline `environment.cpp` path (it
already prints per-stage timings from `processPointClouds.cpp`).

## Results (bag: `20260713Concrete2m-1m_range_2026-07-13-16-51-00.bag`, looped)

| metric | before fix | after fix |
|---|---|---|
| VoxelGrid overflow warnings | frequent | none |
| running avg processing time | ~17.5 ms/frame | ~7-9 ms/frame |
| running avg rate | ~57 Hz | ~114-147 Hz |
| frames with >=1 detected object within 2m | not measured | 357/357 (100%) |
| avg clusters per frame | not measured | ~2.3 |

The ~2.3 clusters/frame (vs. 1 expected for a single person) suggests some
residual non-person fragments (e.g. wall/floor edge artifacts) are still
passing the cluster-size filter — worth a follow-up pass if a clean
single-object output is needed. The `egoRadius` fix specifically targeted
near-field/mount returns, but wasn't isolated in this test run (both changes
were applied together before the bag was replayed).

## `egoRadius` A/B test (same bag, `environment_ros`, everything else fixed)

Ran the bag twice through `environment_ros`, once built with `egoRadius = 0.5`
and once with `egoRadius = 0.0`, capturing per-frame JSON to
`results/ego_on/` and `results/ego_off/` (435 frames each, same bag stamps).

| metric | egoRadius=0.5 | egoRadius=0.0 |
|---|---|---|
| avg clusters/frame | 2.294 | 2.297 |
| frames identical cluster count | 269 / 435 | |
| frames off > on / off < on | 87 / 79 (roughly even split, not systematic) | |

Directly checked the raw `/velodyne_points` data: **zero points fall within
0.5m of the sensor origin in any sampled frame** — the VLP-16's own minimum
sensing range (plus the tripod mount geometry) already keeps returns outside
that radius, so `egoRadius=0.5` is currently a no-op safety net rather than
an active fix. It's left enabled (harmless, and protects against a closer
mount/different rig in the future) but it is **not** what fixed the
intermittent detection — that was entirely the crop-before-voxelize
ordering fix above. The ~2.3 avg clusters/frame (vs. 1 expected) persists
unchanged either way, so those extra fragments are coming from somewhere
else (plane-segmentation leftovers or wall edges within the crop box, not
near-origin noise) and remain a follow-up item.

## Follow-up: wider ROI + distance readout + ground-plane tightening

After the initial close-range tuning above, the crop box was widened for a
different test scenario: `minPoint`/`maxPoint` moved from the tight
`(-2.5,-2.5,-1)`/`(2.5,2.5,1.5)` box to `(-1,-10,-1)`/`(20,10,3)` (a forward-
looking 21m x 20m x 4m volume, `x` starting 1m behind the sensor rather than
symmetric). Tested against the same bag:
- An intermediate `y in [-15,15]` version picked up room-wall/floor
  artifacts as false clusters at ~11m and ~15m (avg 2.7 clusters/frame) and
  cropped the tripod-adjacent person entirely, since they sit at `x~=-1.3`,
  just outside the new `x>=-1` boundary - accepted as intentional for this
  range's use case.
- Narrowing to `y in [-10,10]` dropped those far-field wall clusters (avg
  clusters/frame 2.7 -> 1.15), leaving one consistent nearby cluster.

Separately, RViz was observed showing mostly green points (the `Ground`
display's fixed color per `rviz/lidar_obstacle_detection.rviz`) where the
person should be, meaning too much of the person was being absorbed into
the RANSAC ground-plane fit. Tightening `distanceThreshold` from `0.08` to
`0.03` raised avg detected-object height (`scale.z`) from 0.47m to 0.77m
with no speed regression (~270 Hz avg); pushing further to `0.02` gave only
marginal additional gain (0.77m -> 0.78m), so `0.03` was kept as the
settled value.

Added a live distance readout in `environment_ros.cpp`: a green
`TEXT_VIEW_FACING` RViz marker above each box showing its distance to the
sensor origin, plus a `~box_distances` (`std_msgs/Float32MultiArray`) topic
publishing the same values numerically, in box order.

Current settled hyperparameters (`src/city_block.h`): `filterRes=0.05`,
`minPoint=(-1,-10,-1)`, `maxPoint=(20,10,3)`, `egoRadius=0.5`, `maxIter=40`,
`distanceThreshold=0.03`, `clusterTolerance=0.3`, `minClusterSize=5`,
`maxClusterSize=2000`.

## Tooling added
- `src/environment_ros.cpp`: `~output_dir` param dumps per-frame detected
  boxes as SUSTechPOINTS-schema JSON (`psr.position/rotation/scale`), named
  `<sec>.<nsec>.json` to match ROS header stamps, for comparison against
  hand-labelled ground truth.
- `scripts/evaluate_detections.py`: computes precision/recall/F1/mean center
  error between a predicted-JSON directory and a ground-truth-JSON
  directory, restricted to objects within a configurable range (default 2m)
  of the sensor origin, matching frames by nearest timestamp and boxes by
  nearest center distance.
- `results/20260713Concrete2m-1m_range*/`: JSON outputs from running
  `environment_ros` against the bag, before/after this tuning pass.
