// Shared filter->segment->cluster->box->render pipeline, used by both the
// offline .pcd-file entry point (environment.cpp) and the live ROS/VLP-16
// entry point (environment_ros.cpp).
#ifndef CITY_BLOCK_H_
#define CITY_BLOCK_H_

#include "render/render.h"
#include "processPointClouds.h"
// templates for processPointClouds defined in .cpp file
#include "processPointClouds.cpp"
#include "detection_params.h"

struct DetectionResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters;
  std::vector<Box> boxes;
  // The crop-box ROI actually used for this frame (see minPoint/maxPoint
  // below), so callers can visualize it without duplicating the hardcoded
  // bounds - ground/obstacle separation only ever runs inside this box.
  Box roi;
};

// Filter->segment->cluster->box pipeline, no rendering. Shared by cityBlock()
// (PCL-viewer rendering, used by the offline .pcd entry point) and
// environment_ros.cpp (RViz publishing, used by the live entry point).
inline DetectionResult detectObstacles(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                                        bool applyFilter = true, float filterRes = 0.4, bool removeRoof = true,
                                        bool removeGround = true)
{
  // hyperparameters
  // filter params (filterRes, removeRoof: see function args above)
  Eigen::Vector4f minPoint = DetectionParams::roiMin();
  Eigen::Vector4f maxPoint = DetectionParams::roiMax();
  // segment params
  int maxIter = DetectionParams::ransacMaxIterations;
  float distanceThreshold = DetectionParams::ransacDistanceThreshold;
  // cluster params
  float clusterTolerance = 0.5;
  int minClusterSize = 10;
  int maxClusterSize = 140;

  auto startTime = std::chrono::steady_clock::now();

  // Filter cloud (voxel downsample + crop-box ROI), to reduce computational
  // cost. Skippable via applyFilter for A/B timing/quality comparisons.
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = applyFilter
      ? pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint, removeRoof)
      : inputCloud;
  std::cout << "input cloud size " << inputCloud->points.size()
             << ", post-filter size " << filterCloud->points.size() << std::endl;

  // RansacPlaneSegment needs >=3 points to define a plane hypothesis: below
  // that, Ransac3d's `rand()%num_points` divides by zero (UB - on aarch64
  // this doesn't trap, it silently returns an out-of-range index that then
  // segfaults indexing the point vector) or, for 1-2 points, spins forever
  // trying to pick 3 distinct indices that don't exist. Bail out before
  // that rather than relying on callers to never send a near-empty frame.
  // Only applies when we're about to run RANSAC below - EuclideanClustering
  // handles small/empty clouds fine on its own.
  if (removeGround && filterCloud->points.size() < 3)
  {
    DetectionResult empty;
    empty.groundCloud = filterCloud;
    empty.roi = Box{minPoint.x(), minPoint.y(), minPoint.z(), maxPoint.x(), maxPoint.y(), maxPoint.z()};
    return empty;
  }

  // Step 1. Segment the filtered cloud into ground vs. obstacles - unless
  // ground was already stripped upstream (e.g. by range_gate in the live
  // fusion pipeline, which now removes ground once on the full input cloud
  // before splitting into near/far), in which case there's nothing left to
  // segment and the whole filtered cloud is candidate-obstacle points.
  pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud;
  if (removeGround)
  {
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlaneSegment(filterCloud, maxIter, distanceThreshold);
    groundCloud = segmentCloud.second;
    obstacleCloud = segmentCloud.first;
  }
  else
  {
    obstacleCloud = filterCloud;
  }

  // Step 2. Cluster the obstacle cloud.
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(obstacleCloud, clusterTolerance, minClusterSize, maxClusterSize);

  DetectionResult result;
  result.groundCloud = groundCloud;
  result.obstacleClusters = cloudClusters;
  result.roi = Box{minPoint.x(), minPoint.y(), minPoint.z(), maxPoint.x(), maxPoint.y(), maxPoint.z()};

  // Step 3. Find bounding boxes for the clusters
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        result.boxes.push_back(pointProcessorI->BoundingBox(cluster));
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "detectObstacles total (filter=" << (applyFilter ? "on" : "off")
             << ") took " << elapsedTime.count() << " milliseconds" << std::endl;

  return result;
}

inline void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  DetectionResult result = detectObstacles(pointProcessorI, inputCloud);

  renderPointCloud(viewer,result.groundCloud,"planeCloud",Color(0,1,0));

  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(size_t clusterId = 0; clusterId < result.obstacleClusters.size(); ++clusterId)
  {
        renderPointCloud(viewer,result.obstacleClusters[clusterId],"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        renderBox(viewer,result.boxes[clusterId],(int)clusterId);
  }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
inline void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

#endif /* CITY_BLOCK_H_ */
