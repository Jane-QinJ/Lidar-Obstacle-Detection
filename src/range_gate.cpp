// Splits one incoming sensor_msgs/PointCloud2 stream into two, by each
// point's Euclidean distance from the sensor origin:
//   - "near" (< range_threshold, default 2m): fed to the clustering-based
//     lidar_obstacle_detection node (environment_ros), which is cheap and
//     robust at close range where a deep-learning detector's training
//     distribution/voxelization is a poor fit and there are too few
//     returns for a reliable inference.
//   - "far"  (>= range_threshold): fed to the PointPillars deep-learning
//     detector, which needs more points/context per object and is tuned
//     for middle/long range.
// This keeps both detectors from ever double-detecting the same object in
// the overlap and lets each one be tuned independently for its own range.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace
{
    ros::Publisher nearPub;
    ros::Publisher farPub;
    double rangeThreshold;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI> inputCloud;
        pcl::fromROSMsg(*msg, inputCloud);

        pcl::PointCloud<pcl::PointXYZI> nearCloud;
        pcl::PointCloud<pcl::PointXYZI> farCloud;
        nearCloud.reserve(inputCloud.size());
        farCloud.reserve(inputCloud.size());

        const double thresholdSq = rangeThreshold * rangeThreshold;
        for (const auto& point : inputCloud.points)
        {
            const double distSq = static_cast<double>(point.x) * point.x
                                 + static_cast<double>(point.y) * point.y
                                 + static_cast<double>(point.z) * point.z;
            if (distSq < thresholdSq)
                nearCloud.push_back(point);
            else
                farCloud.push_back(point);
        }

        nearCloud.header = inputCloud.header;
        farCloud.header = inputCloud.header;

        sensor_msgs::PointCloud2 nearMsg;
        pcl::toROSMsg(nearCloud, nearMsg);
        nearMsg.header = msg->header;
        nearPub.publish(nearMsg);

        sensor_msgs::PointCloud2 farMsg;
        pcl::toROSMsg(farCloud, farMsg);
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

    ros::Subscriber sub = nh.subscribe(inputTopic, 1, cloudCallback);
    ROS_INFO_STREAM("range_gate: splitting " << inputTopic << " at " << rangeThreshold
        << "m -> near: " << nearTopic << " (clustering), far: " << farTopic << " (PointPillars)");

    ros::spin();
    return 0;
}
