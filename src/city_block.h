// Shared filter->segment->cluster->box->render pipeline, used by both the
// offline .pcd-file entry point (environment.cpp) and the live ROS/VLP-16
// entry point (environment_ros.cpp).
#ifndef CITY_BLOCK_H_
#define CITY_BLOCK_H_

#include "render/render.h"
#include "processPointClouds.h"
// templates for processPointClouds defined in .cpp file
#include "processPointClouds.cpp"

struct DetectionResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters;
  std::vector<Box> boxes;
};

// Filter->segment->cluster->box pipeline, no rendering. Shared by cityBlock()
// (PCL-viewer rendering, used by the offline .pcd entry point) and
// environment_ros.cpp (RViz publishing, used by the live entry point).
inline DetectionResult detectObstacles(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // hyperparameters
  // Tuned for stable single-person detection within a ~2m range (close-range
  // static rig test), not the original highway-driving scene: the crop box
  // is tight around the sensor, the voxel leaf is much finer since a person
  // at 2m is a small, sparse cluster that a 0.4m voxel was collapsing/losing
  // (this was the cause of inconsistent detection), and min/max cluster size
  // are widened to match the resulting higher point density per person.
  // filter params
  float filterRes = 0.05;
  Eigen::Vector4f minPoint(-1, -10.0, -1.0, 1);
  Eigen::Vector4f maxPoint(20.0, 10.0, 3.0, 1);
  // radius (m) around the sensor origin to discard outright - tripod/mount
  // near-field returns, not real obstacles
  float egoRadius = 0.5;
  // segment params
  int maxIter = 40;
  float distanceThreshold = 0.03;
  // cluster params
  float clusterTolerance = 0.3;
  int minClusterSize = 5;
  int maxClusterSize = 2000;

  // Filter cloud, to reduce omputational cost
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint, egoRadius);

  // Step 1. Segment the filtered cloud into two parts, road and obstacles.
  // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIter, distanceThreshold);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlaneSegment(filterCloud, maxIter, distanceThreshold);

  // Step 2. Cluster the obstacle cloud.
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

  DetectionResult result;
  result.groundCloud = segmentCloud.second;
  result.obstacleClusters = cloudClusters;

  // Step 3. Find bounding boxes for the clusters
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        result.boxes.push_back(pointProcessorI->BoundingBox(cluster));
  }

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
