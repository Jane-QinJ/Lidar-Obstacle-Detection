// Live entry point: subscribes to a sensor_msgs/PointCloud2 topic (e.g. a
// Velodyne VLP-16 publishing via the ROS "velodyne" driver stack), runs each
// incoming frame through the same detectObstacles() pipeline used by the
// offline .pcd playback in environment.cpp, and publishes the result as
// topics for RViz (point clouds + bounding-box markers) instead of opening a
// local PCL viewer window - this keeps the node usable headless (e.g. over
// SSH on a robot with no display), with visualization done remotely in RViz.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "city_block.h"

namespace
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI;
    ros::Publisher groundPub;
    ros::Publisher obstaclePub;
    ros::Publisher markerPub;
    bool applyFilter = true;
    float filterRes = 0.15;
    bool removeRoof = false;

    visualization_msgs::Marker boxToMarker(const Box& box, int id, const std_msgs::Header& header)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "detection_boxes";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.color.r = 1.0f;
        marker.color.a = 1.0f;

        geometry_msgs::Point corners[8];
        corners[0].x = box.x_min; corners[0].y = box.y_min; corners[0].z = box.z_min;
        corners[1].x = box.x_max; corners[1].y = box.y_min; corners[1].z = box.z_min;
        corners[2].x = box.x_max; corners[2].y = box.y_max; corners[2].z = box.z_min;
        corners[3].x = box.x_min; corners[3].y = box.y_max; corners[3].z = box.z_min;
        corners[4].x = box.x_min; corners[4].y = box.y_min; corners[4].z = box.z_max;
        corners[5].x = box.x_max; corners[5].y = box.y_min; corners[5].z = box.z_max;
        corners[6].x = box.x_max; corners[6].y = box.y_max; corners[6].z = box.z_max;
        corners[7].x = box.x_min; corners[7].y = box.y_max; corners[7].z = box.z_max;

        const int edges[12][2] = {
            {0,1}, {1,2}, {2,3}, {3,0},
            {4,5}, {5,6}, {6,7}, {7,4},
            {0,4}, {1,5}, {2,6}, {3,7},
        };
        for (const auto& edge : edges)
        {
            marker.points.push_back(corners[edge[0]]);
            marker.points.push_back(corners[edge[1]]);
        }
        return marker;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *inputCloudI);

        DetectionResult result = detectObstacles(pointProcessorI, inputCloudI, applyFilter, filterRes, removeRoof);

        sensor_msgs::PointCloud2 groundMsg;
        pcl::toROSMsg(*result.groundCloud, groundMsg);
        groundMsg.header = msg->header;
        groundPub.publish(groundMsg);

        pcl::PointCloud<pcl::PointXYZI> obstacleCloud;
        for (const auto& cluster : result.obstacleClusters)
            obstacleCloud += *cluster;
        sensor_msgs::PointCloud2 obstacleMsg;
        pcl::toROSMsg(obstacleCloud, obstacleMsg);
        obstacleMsg.header = msg->header;
        obstaclePub.publish(obstacleMsg);

        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        markerArray.markers.push_back(clearMarker);
        for (size_t i = 0; i < result.boxes.size(); ++i)
            markerArray.markers.push_back(boxToMarker(result.boxes[i], (int)i, msg->header));
        markerPub.publish(markerArray);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    std::string topic;
    privateNh.param<std::string>("topic", topic, "/velodyne_points");
    privateNh.param<bool>("filter_cloud", applyFilter, true);
    privateNh.param<float>("filter_res", filterRes, 0.15f);
    privateNh.param<bool>("remove_roof", removeRoof, false);

    pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    groundPub = privateNh.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
    obstaclePub = privateNh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    markerPub = privateNh.advertise<visualization_msgs::MarkerArray>("detection_boxes", 1);

    ros::Subscriber sub = nh.subscribe(topic, 1, cloudCallback);
    ROS_INFO_STREAM("subscribed to " << topic << "; filter_cloud=" << (applyFilter ? "true" : "false")
        << ", filter_res=" << filterRes << "m, remove_roof=" << (removeRoof ? "true" : "false")
        << "; publishing detections on "
        << privateNh.resolveName("ground_cloud") << ", "
        << privateNh.resolveName("obstacle_cloud") << ", "
        << privateNh.resolveName("detection_boxes")
        << " - view in rviz with Fixed Frame set to the input cloud's frame_id");

    ros::spin();

    delete pointProcessorI;
    return 0;
}
