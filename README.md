# Lidar Obstacle Detection

Project for Udacity's Sensor Fusion Engineer Nanodegree Program

**Project Goals**

* Implement Obstacle detection on real PCD from a lidar
* use [pcl-library](http://pointclouds.org/) for general data handling and initial testing
* implement following modules:
  * PCD filtering, for reducing computational cost, without loss of detail
  * Segment the filtered cloud into two parts, road and obstacles, using RANSAC based 3D-plane extraction
  * Cluster the obstacle cloud, using K-D Tree for 3D space.
  * Find bounding boxes for the clusters


### Dependencies:

Verified working on:

* Ubuntu 20.04 OS, PCL 1.8.1 (`sudo apt install libpcl-dev`)
* cmake >= 2.8
* gcc/g++ with C++11 support

Target deployment platform: Jetson Orin Nano (JetPack, aarch64). The build only
depends on PCL/CMake with no x86-specific code, so it is expected to build the
same way there via `libpcl-dev`; re-verify on-device before relying on it.

### Sample Data

Point cloud `.pcd` sample data is **not** version-controlled (see `.gitignore`)
to keep the repository small for deployment to resource-constrained targets.

* `streamPcd()` (called from `main()` in `environment.cpp`) expects a sequence
  of ordered `.pcd` files under `src/sensors/data/pcd/data_1/`.
* The original 22-frame Udacity sample sequence is still recoverable from git
  history: `git show 65415fb:src/sensors/data/pcd/data_1/0000000000.pcd > out.pcd` (etc.), or `git checkout 65415fb -- src/sensors/data/pcd/data_1/`.
* Alternatively, point `data_1/` at your own recorded `.pcd` sequence.


### Notes on some files & folders

* README.md: this file.
* **images** - folder with images for the readme-file
* **./src/**
  * **environment.cpp** - main function
  * **ransac.cpp** - function for RANSAC-based segmentation implementation
  * **cluster_kdtree.cpp** & **kdtree_pcl.h** - functions for KD-Tree based clustering implementation
  * **processPointClouds.cpp** & **processPointClouds.h** - functions for point-cloud processing. functions that use segmentation and clustering based on PCL-library are also present, but commented
  * **/quiz/...** - contains quiz functions for testing ransac and clustering implementation
  * **/render/...** - contains rendering functions for display
  * **/sensors/..** - contains point-cloud-data files and functions for use with synthetic data.

### Build and Run

clone this repository, enter the cloned directory/folder and build:

```
mkdir build && cd build
cmake ..
make
```

to run, use following from within the build folder:

```
./environment
```

### Sample Results

##### Quiz-Ransac

To test performance of 2D and 3D RANSAC implementation, build:

```
cd src/quiz/ransac/
mkdir build && cd build
cmake ..
make
```

to view result of 3D RANSAC implementation, launch

```
./qizRansac3d
```

a sample result is shown below, where road-plane in highlighted in green and objects on road in red.

3D RANSAC sample image:
![alt text](/images/ransac3d.jpg)

##### KD-Tree / Euclidean Clustering

To test performance of KD-Tree implementation on 2D sample points, build:

```
cd src/quiz/cluster/
mkdir build && cd build
cmake ..
make
```

to view result of 3D RANSAC implementation, launch

```
./qizCluster
```

a sample result is shown below.

2D KD-Tree implementation sample:
![alt text](/images/kdtree_sample.jpg)

##### Lidar-Obstacle-Detection

once `./environment` is launch pcd data is read from files at `/sensors/data/pcd/data_1/` and plotted after filtering-segmentation-clustering, as shown in sample image below.

sample lidar-obstacle-detection image:
![alt text](/images/lidar_obs_det_01.jpg)

##### Validation on synthetic data

To confirm the segmentation/clustering pipeline generalizes beyond the bundled
sample sequence, it was also run against a synthetic point cloud (ground plane
+ 3 well-separated obstacle blobs) built independently of any repo data. Both
the hand-rolled KD-tree clustering and PCL's built-in `EuclideanClusterExtraction`
correctly recovered all 3 clusters:

![alt text](/images/custom_data_test.png)

#### Resources

* To install PCL, C++ https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/
