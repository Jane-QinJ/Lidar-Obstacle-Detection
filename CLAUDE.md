# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

C++ lidar obstacle detection pipeline (Udacity Sensor Fusion Nanodegree project), targeting deployment on a Jetson Orin Nano. Reads a stream of `.pcd` point-cloud frames, filters/downsamples them, segments road plane vs. obstacles via RANSAC, clusters obstacle points via a hand-rolled 3D KD-tree, computes bounding boxes, and renders the result with PCL's viewer.

## Build and Run

Main pipeline:
```
mkdir build && cd build
cmake ..
make
./environment          # run from within build/, expects ../src/sensors/data/pcd/data_1
```

There is no test suite, linter, or CI config in this repo — verification is visual, via the PCL viewer.

Quiz/demo executables (small standalone RANSAC and clustering demos, each with their own `CMakeLists.txt`):
```
cd src/quiz/ransac/ && mkdir build && cd build && cmake .. && make && ./quizRansac3d
cd src/quiz/cluster/ && mkdir build && cd build && cmake .. && make && ./quizCluster
```

Dependencies: PCL 1.8.1 (`sudo apt install libpcl-dev`), cmake, gcc/g++ with C++11. No x86-specific code, so it should build on aarch64/Jetson the same way, but re-verify on-device before relying on that.

### Sample data

`.pcd` point cloud files are gitignored (kept out of the repo to stay lightweight for deployment) — `src/sensors/data/pcd/data_1/` will be empty on a fresh checkout. `streamPcd()` in `environment.cpp`'s `main()` expects an ordered sequence of `.pcd` files there. The original 22-frame sample sequence is recoverable from git history: `git checkout 65415fb -- src/sensors/data/pcd/data_1/`. Otherwise, point `data_1/` at your own recorded sequence, or use the bundled `src/sensors/data/simpleHighway.pcd` for the synthetic `simpleHighway()` path.

## Architecture

**Everything is a header-only template pipeline.** `ProcessPointClouds<PointT>` (`processPointClouds.h`/`.cpp`) is templated on the PCL point type so the same code path works for `pcl::PointXYZ` (synthetic data) and `pcl::PointXYZI` (real lidar frames with intensity). Because it's a template class, `processPointClouds.h` `#include`s `ransac.cpp` and `cluster_kdtree.cpp` directly (not just their headers) — and `environment.cpp` in turn `#include`s `processPointClouds.cpp` directly, since template definitions must be visible at the instantiation site. Keep this include chain in mind when adding new template methods: they need to live in the `.cpp`/`.h`, not a separately-compiled `.cpp`.

**Pipeline stages, each with two implementations you can swap between** (see `cityBlock()` in `environment.cpp`, which picks the "local"/hand-rolled variants):
1. **Filter** — `ProcessPointClouds::FilterCloud`: PCL voxel-grid downsampling + crop-box region-of-interest + roof-point removal (hardcoded ego-vehicle roof box).
2. **Segment** (road plane vs. obstacles) — either PCL's `SegmentPlane` (`pcl::SACSegmentation`) or the hand-rolled `RansacPlaneSegment`, which delegates to `Ransac<PointT>` in `ransac.cpp` (3-point plane RANSAC using `unordered_set` for inlier tracking).
3. **Cluster** — either PCL's `Clustering` (`pcl::EuclideanClusterExtraction`) or the hand-rolled `EuclideanClustering`, which delegates to `ClusterPts<PointT>` in `cluster_kdtree.cpp`. `ClusterPts` builds a custom 3D `KdTree` (`kdtree_pcl.h`, fixed point type `pcl::PointXYZI` — not templated like the rest of the pipeline) and does recursive proximity-based Euclidean clustering via depth-first search, cycling the split axis through x/y/z by tree depth.
4. **Bounding box** — `BoundingBox()` via `pcl::getMinMax3D`.

Note `kdtree_pcl.h` (used by the main pipeline, 3D, `pcl::PointXYZI`-specific) is a different implementation from `src/quiz/cluster/kdtree.h` (2D quiz version, generic points) — don't conflate the two when touching KD-tree logic.

**Entry point** (`environment.cpp`): `main()` sets up the PCL viewer/camera, then loops over `streamPcd()`-discovered `.pcd` files, running `cityBlock()` per frame (filter → segment → cluster → box → render) and re-rendering each iteration. `simpleHighway()` is an alternate, currently-unused entry path that exercises the pipeline against a synthetic scene (`sensors/lidar.h`'s ray-cast simulator over hardcoded `Car` boxes) instead of streamed `.pcd` files — both are wired up but only the streaming path is active in `main()`.

Rendering (`render/render.cpp`, `render/render.h`, `render/box.h`) is a thin wrapper over `pcl::visualization::PCLVisualizer` for point clouds, rays, boxes, and highway/car geometry — not pipeline logic.

**Hyperparameters** (voxel leaf size, crop-box bounds, RANSAC iterations/distance threshold, cluster tolerance/min/max size) are hardcoded locally in `cityBlock()`, not centralized in a config — check there first when tuning detection behavior.
