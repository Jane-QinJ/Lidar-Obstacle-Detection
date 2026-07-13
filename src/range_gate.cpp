// Crops the incoming sensor_msgs/PointCloud2 to the shared detection ROI
// (DetectionParams), strips its ground plane (same
// horizontal-plane-constrained RANSAC the near-range path uses), and only
// then splits the remaining obstacle-only cloud in two, by each point's
// Euclidean distance from the sensor origin:
//   - "near" (< range_threshold, default 2m): fed to the clustering-based
//     lidar_obstacle_detection node (environment_ros), which is cheap and
//     robust at close range where a deep-learning detector's training
//     distribution/voxelization is a poor fit and there are too few
//     returns for a reliable inference. Since ground is already gone by the
//     time it gets here, that node is run with remove_ground:=false so it
//     doesn't try to re-segment ground out of a cloud that no longer has
//     any (see lidar_fusion.launch).
//   - "far"  (>= range_threshold): fed to the PointPillars deep-learning
//     detector, which needs more points/context per object and is tuned
//     for middle/long range.
// This keeps both detectors from ever double-detecting the same object in
// the overlap, lets each one be tuned independently for its own range, and
// - by cropping/ground-removing once upstream of the split instead of once
// per branch - means "the ROI" and "what counts as ground" are decided
// exactly once per frame instead of twice (previously PointPillars wasn't
// ROI-cropped or ground-removed at all, and the near path re-derived both
// independently downstream in city_block.h).
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

#include "detection_params.h"
#include "ransac.cpp"

namespace
{
    ros::Publisher nearPub;
    ros::Publisher farPub;
    ros::Publisher groundPub;
    double rangeThreshold;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropToRoi(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> roi;
        roi.setMin(DetectionParams::roiMin());
        roi.setMax(DetectionParams::roiMax());
        roi.setInputCloud(cloud);
        roi.filter(*cropped);
        return cropped;
    }

    // Strip ground-plane points from the (already ROI-cropped) input cloud,
    // returning {obstacles, ground} - the ground half exists purely so
    // callers can publish/visualize what got removed; nothing downstream of
    // range_gate ever sees it otherwise now that ground removal happens
    // here instead of per-branch. See detectObstacles()'s empty-cloud guard
    // in city_block.h for why <3 points must bail before RANSAC (rand()%0
    // UB / infinite loop).
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
    removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
        if (cloud->points.size() < 3)
            return {cloud, ground};

        Ransac<pcl::PointXYZI> ransac(DetectionParams::ransacMaxIterations,
                                       DetectionParams::ransacDistanceThreshold,
                                       cloud->points.size());
        std::unordered_set<int> groundInliers = ransac.Ransac3d(cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>);
        obstacles->points.reserve(cloud->points.size() - groundInliers.size());
        ground->points.reserve(groundInliers.size());
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (groundInliers.count(static_cast<int>(i)))
                ground->points.push_back(cloud->points[i]);
            else
                obstacles->points.push_back(cloud->points[i]);
        }
        obstacles->width = obstacles->points.size();
        obstacles->height = 1;
        ground->width = ground->points.size();
        ground->height = 1;
        return {obstacles, ground};
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI> inputCloudMsg;
        pcl::fromROSMsg(*msg, inputCloudMsg);
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>(inputCloudMsg));

        // Crop and ground-remove once, on the whole input, before splitting -
        // so both downstream detectors see the same ROI and the same
        // definition of "ground" instead of each deriving it independently.
        pcl::PointCloud<pcl::PointXYZI>::Ptr roiCloud = cropToRoi(inputCloud);
        auto groundSplit = removeGround(roiCloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud = groundSplit.first;
        pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud = groundSplit.second;

        groundCloud->header = inputCloudMsg.header;
        sensor_msgs::PointCloud2 groundMsg;
        pcl::toROSMsg(*groundCloud, groundMsg);
        groundMsg.header = msg->header;
        groundPub.publish(groundMsg);

        pcl::PointCloud<pcl::PointXYZI>::Ptr nearCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr farCloud(new pcl::PointCloud<pcl::PointXYZI>);
        nearCloud->reserve(obstacleCloud->size());
        farCloud->reserve(obstacleCloud->size());

        const double thresholdSq = rangeThreshold * rangeThreshold;
        for (const auto& point : obstacleCloud->points)
        {
            const double distSq = static_cast<double>(point.x) * point.x
                                 + static_cast<double>(point.y) * point.y
                                 + static_cast<double>(point.z) * point.z;
            if (distSq < thresholdSq)
                nearCloud->push_back(point);
            else
                farCloud->push_back(point);
        }

        nearCloud->header = inputCloudMsg.header;
        farCloud->header = inputCloudMsg.header;

        sensor_msgs::PointCloud2 nearMsg;
        pcl::toROSMsg(*nearCloud, nearMsg);
        nearMsg.header = msg->header;
        nearPub.publish(nearMsg);

        sensor_msgs::PointCloud2 farMsg;
        pcl::toROSMsg(*farCloud, farMsg);
        farMsg.header = msg->header;
        farPub.publish(farMsg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_gate");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    std::string inputTopic, nearTopic, farTopic;
    privateNh.param<std::string>("input_topic", inputTopic, "/velodyne_points");
    privateNh.param<std::string>("near_topic", nearTopic, "/velodyne_points/near");
    privateNh.param<std::string>("far_topic", farTopic, "/velodyne_points/far");
    privateNh.param<double>("range_threshold", rangeThreshold, 2.0);

    nearPub = nh.advertise<sensor_msgs::PointCloud2>(nearTopic, 1);
    farPub = nh.advertise<sensor_msgs::PointCloud2>(farTopic, 1);
    groundPub = privateNh.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);

    ros::Subscriber sub = nh.subscribe(inputTopic, 1, cloudCallback);
    ROS_INFO_STREAM("range_gate: splitting " << inputTopic << " at " << rangeThreshold
        << "m -> near: " << nearTopic << " (clustering), far: " << farTopic << " (PointPillars); "
        << "removed ground points published on " << privateNh.resolveName("ground_cloud"));

    ros::spin();
    return 0;
}
