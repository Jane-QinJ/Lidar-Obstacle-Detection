# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

C++ lidar obstacle detection pipeline (Udacity Sensor Fusion Nanodegree project), targeting deployment on a Jetson Orin Nano. Consumes `.pcd` point-cloud frames (offline playback) or a live `sensor_msgs/PointCloud2` ROS1 topic (e.g. a Velodyne VLP-16), filters/downsamples them, segments road plane vs. obstacles via RANSAC, clusters obstacle points via a hand-rolled 3D KD-tree, and computes bounding boxes. The offline path renders the result with PCL's own viewer; the live ROS1 path publishes it as topics for visualization in RViz instead.

## Build and Run

Offline `.pcd`-file playback:
```
mkdir build && cd build
cmake ..
make
./environment          # run from within build/, expects ../src/sensors/data/pcd/data_1
```

Live ROS1 input (VLP-16 or any `sensor_msgs/PointCloud2` publisher) — source ROS *before* `cmake` so the `environment_ros` target gets configured:
```
source /opt/ros/noetic/setup.bash
mkdir build && cd build
cmake ..
make                    # now also builds environment_ros
roscore &               # or roslaunch the velodyne driver, which also brings up a master
./environment_ros                       # subscribes to /velodyne_points by default
./environment_ros _topic:=/my/topic     # override via ROS remapping arg
rviz -d rviz/lidar_obstacle_detection.rviz   # visualize the published detections
```

There is no test suite, linter, or CI config in this repo — verification is visual, via the PCL viewer (offline) or RViz (live ROS1).

Quiz/demo executables (small standalone RANSAC and clustering demos, each with their own `CMakeLists.txt`):
```
cd src/quiz/ransac/ && mkdir build && cd build && cmake .. && make && ./quizRansac3d
cd src/quiz/cluster/ && mkdir build && cd build && cmake .. && make && ./quizCluster
```

Dependencies: PCL 1.8.1+ (`sudo apt install libpcl-dev`), cmake, gcc/g++ with C++14. For live ROS input: ROS1 Noetic with `roscpp`, `sensor_msgs`, and `pcl_conversions` (`sudo apt install ros-noetic-ros-base ros-noetic-pcl-conversions`), plus a Velodyne driver (`ros-noetic-velodyne`) to actually talk to a VLP-16. No x86-specific code, so it should build on aarch64/Jetson the same way, but re-verify on-device before relying on that.

### Sample data

`.pcd` point cloud files are gitignored (kept out of the repo to stay lightweight for deployment) — `src/sensors/data/pcd/data_1/` will be empty on a fresh checkout. `streamPcd()` in `environment.cpp`'s `main()` expects an ordered sequence of `.pcd` files there. The original 22-frame sample sequence is recoverable from git history: `git checkout 65415fb -- src/sensors/data/pcd/data_1/`. Otherwise, point `data_1/` at your own recorded sequence, or use the bundled `src/sensors/data/simpleHighway.pcd` for the synthetic `simpleHighway()` path.

## Architecture

**Everything is a header-only template pipeline.** `ProcessPointClouds<PointT>` (`processPointClouds.h`/`.cpp`) is templated on the PCL point type so the same code path works for `pcl::PointXYZ` (synthetic data) and `pcl::PointXYZI` (real lidar frames with intensity). Because it's a template class, `processPointClouds.h` `#include`s `ransac.cpp` and `cluster_kdtree.cpp` directly (not just their headers) — and anything that instantiates it (`city_block.h`) in turn `#include`s `processPointClouds.cpp` directly, since template definitions must be visible at the instantiation site. Keep this include chain in mind when adding new template methods: they need to live in the `.cpp`/`.h`, not a separately-compiled `.cpp`.

**Pipeline stages, each with two implementations you can swap between** (see `cityBlock()` in `city_block.h`, which picks the "local"/hand-rolled variants):
1. **Filter** — `ProcessPointClouds::FilterCloud`: PCL voxel-grid downsampling + crop-box region-of-interest + roof-point removal (hardcoded ego-vehicle roof box).
2. **Segment** (road plane vs. obstacles) — either PCL's `SegmentPlane` (`pcl::SACSegmentation`) or the hand-rolled `RansacPlaneSegment`, which delegates to `Ransac<PointT>` in `ransac.cpp` (3-point plane RANSAC using `unordered_set` for inlier tracking).
3. **Cluster** — either PCL's `Clustering` (`pcl::EuclideanClusterExtraction`) or the hand-rolled `EuclideanClustering`, which delegates to `ClusterPts<PointT>` in `cluster_kdtree.cpp`. `ClusterPts` builds a custom 3D `KdTree` (`kdtree_pcl.h`, fixed point type `pcl::PointXYZI` — not templated like the rest of the pipeline) and does recursive proximity-based Euclidean clustering via depth-first search, cycling the split axis through x/y/z by tree depth.
4. **Bounding box** — `BoundingBox()` via `pcl::getMinMax3D`.

Note `kdtree_pcl.h` (used by the main pipeline, 3D, `pcl::PointXYZI`-specific) is a different implementation from `src/quiz/cluster/kdtree.h` (2D quiz version, generic points) — don't conflate the two when touching KD-tree logic.

**Two entry points share one pipeline via `city_block.h`.** The actual filter → segment → cluster → box computation lives in `detectObstacles()` in `src/city_block.h`, returning a `DetectionResult` (ground cloud + per-cluster obstacle clouds + `Box`es) — it has no rendering/ROS dependency. `cityBlock()` wraps it for the offline path by additionally rendering the result via a PCL viewer. Both `detectObstacles()`/`cityBlock()` and `initCamera()` are `inline` functions in this header so `environment.cpp` and `environment_ros.cpp` each compile their own copy without an ODR violation (they're separate executables, never linked together).
- **`environment.cpp`** (offline): `main()` sets up the PCL viewer/camera, then loops over `streamPcd()`-discovered `.pcd` files, calling `cityBlock()` per frame to render via PCL's viewer. `simpleHighway()`/`initHighway()` are an alternate, currently-unused path that exercises the pipeline against a synthetic scene (`sensors/lidar.h`'s ray-cast simulator over hardcoded `Car` boxes) instead of streamed `.pcd` files.
- **`environment_ros.cpp`** (live): a headless ROS1 node (`ros::spin()`, no local GUI window — so it runs fine over SSH on a robot with no display) that subscribes to a `sensor_msgs/PointCloud2` topic (private param `~topic`, default `/velodyne_points`), converts each message to `pcl::PointCloud<pcl::PointXYZI>` via `pcl::fromROSMsg`, calls `detectObstacles()` directly (not `cityBlock()` — it doesn't want the PCL viewer), and publishes the result as `~/ground_cloud` and `~/obstacle_cloud` (`sensor_msgs/PointCloud2`) plus `~/detection_boxes` (`visualization_msgs/MarkerArray`, one `LINE_LIST` wireframe box per cluster, built by hand from each `Box`'s 8 corners — there's no ROS message type for an axis-aligned box). `rviz/lidar_obstacle_detection.rviz` has a ready-made layout for these three topics (Fixed Frame `velodyne`). It links `roscpp`/`sensor_msgs`/`pcl_conversions`/`visualization_msgs` via `pkg-config` rather than `find_package(catkin ...)`, so it builds as a plain executable with no catkin workspace needed — just `source /opt/ros/noetic/setup.bash` before `cmake`. In `CMakeLists.txt`, this target is conditional on `pkg_check_modules(ROSCPP ...)` succeeding; if ROS isn't sourced, `environment_ros` is silently skipped and only `environment` builds. Keep ROS message headers (`ros/`, `sensor_msgs/`, `visualization_msgs/`) out of `city_block.h`/`render/*` — those are compiled into both executables, and `environment` must keep building without ROS installed.

Rendering (`render/render.cpp`, `render/render.h`, `render/box.h`) is a thin wrapper over `pcl::visualization::PCLVisualizer` for point clouds, rays, boxes, and highway/car geometry — used only by the offline path, not pipeline logic, and not ROS-aware (see above).

**Hyperparameters** (voxel leaf size, crop-box bounds, RANSAC iterations/distance threshold, cluster tolerance/min/max size) are hardcoded in `detectObstacles()` in `city_block.h`, not centralized in a config — check there first when tuning detection behavior. They apply identically to both the offline and live-ROS entry points.
