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
#include <std_msgs/Float32MultiArray.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "city_block.h"

namespace
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI;
    ros::Publisher groundPub;
    ros::Publisher obstaclePub;
    ros::Publisher markerPub;
    ros::Publisher distancePub;
    std::string outputDir;

    // Straight-line distance from the sensor origin to a box's center -
    // i.e. how far the tripod is from whatever's in that box.
    float boxDistance(const Box& box)
    {
        float cx = 0.5f * (box.x_min + box.x_max);
        float cy = 0.5f * (box.y_min + box.y_max);
        float cz = 0.5f * (box.z_min + box.z_max);
        return std::sqrt(cx * cx + cy * cy + cz * cz);
    }

    visualization_msgs::Marker distanceLabelMarker(const Box& box, float distance, int id, const std_msgs::Header& header)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "detection_distances";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5f * (box.x_min + box.x_max);
        marker.pose.position.y = 0.5f * (box.y_min + box.y_max);
        marker.pose.position.z = box.z_max + 0.15f;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.2;
        marker.color.g = 1.0f;
        marker.color.a = 1.0f;
        char text[32];
        std::snprintf(text, sizeof(text), "%.2f m", distance);
        marker.text = text;
        return marker;
    }

    // Writes one JSON label file per frame, in the same box schema used by
    // SUSTechPOINTS (position/rotation/scale under "psr"), so detections can
    // be diffed against hand-labelled ground truth with the same tooling.
    // Boxes here are axis-aligned (rotation always 0) since that's all
    // BoundingBox() produces.
    void writeBoxesJson(const std::vector<Box>& boxes, const std_msgs::Header& header)
    {
        char filename[64];
        std::snprintf(filename, sizeof(filename), "%d.%09d.json", header.stamp.sec, header.stamp.nsec);
        std::ofstream out(outputDir + "/" + filename);
        if (!out)
        {
            ROS_WARN_STREAM_THROTTLE(5, "failed to open output file in " << outputDir);
            return;
        }

        out << "[\n";
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            const Box& b = boxes[i];
            float cx = 0.5f * (b.x_min + b.x_max);
            float cy = 0.5f * (b.y_min + b.y_max);
            float cz = 0.5f * (b.z_min + b.z_max);
            float sx = b.x_max - b.x_min;
            float sy = b.y_max - b.y_min;
            float sz = b.z_max - b.z_min;

            out << "  {\n"
                << "    \"obj_id\": \"" << i << "\",\n"
                << "    \"obj_type\": \"Unknown\",\n"
                << "    \"psr\": {\n"
                << "      \"position\": {\"x\": " << cx << ", \"y\": " << cy << ", \"z\": " << cz << "},\n"
                << "      \"rotation\": {\"x\": 0, \"y\": 0, \"z\": 0},\n"
                << "      \"scale\": {\"x\": " << sx << ", \"y\": " << sy << ", \"z\": " << sz << "}\n"
                << "    }\n"
                << "  }" << (i + 1 < boxes.size() ? "," : "") << "\n";
        }
        out << "]\n";
    }

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

    // Rolling average of detectObstacles() wall-clock time, logged every
    // logInterval frames so detection speed can be reported without
    // depending on external tools like `rostopic hz`.
    double totalProcessingSec = 0.0;
    int frameCount = 0;
    const int logInterval = 30;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *inputCloudI);

        auto t0 = std::chrono::steady_clock::now();
        DetectionResult result = detectObstacles(pointProcessorI, inputCloudI);
        auto t1 = std::chrono::steady_clock::now();
        double frameSec = std::chrono::duration<double>(t1 - t0).count();
        totalProcessingSec += frameSec;
        ++frameCount;
        if (frameCount % logInterval == 0)
        {
            double avgSec = totalProcessingSec / frameCount;
            ROS_INFO_STREAM("detection speed: last frame " << (frameSec * 1000.0) << " ms ("
                << (1.0 / frameSec) << " Hz), running avg " << (avgSec * 1000.0) << " ms ("
                << (1.0 / avgSec) << " Hz) over " << frameCount << " frames");
        }

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

        std_msgs::Float32MultiArray distanceMsg;
        for (size_t i = 0; i < result.boxes.size(); ++i)
        {
            markerArray.markers.push_back(boxToMarker(result.boxes[i], (int)i, msg->header));
            float distance = boxDistance(result.boxes[i]);
            markerArray.markers.push_back(distanceLabelMarker(result.boxes[i], distance, (int)i, msg->header));
            distanceMsg.data.push_back(distance);
        }
        markerPub.publish(markerArray);
        distancePub.publish(distanceMsg);

        if (!outputDir.empty())
            writeBoxesJson(result.boxes, msg->header);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    std::string topic;
    privateNh.param<std::string>("topic", topic, "/velodyne_points");
    privateNh.param<std::string>("output_dir", outputDir, "");
    if (!outputDir.empty())
        ROS_INFO_STREAM("writing per-frame detection boxes as JSON to " << outputDir);

    pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    groundPub = privateNh.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
    obstaclePub = privateNh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    markerPub = privateNh.advertise<visualization_msgs::MarkerArray>("detection_boxes", 1);
    distancePub = privateNh.advertise<std_msgs::Float32MultiArray>("box_distances", 1);

    ros::Subscriber sub = nh.subscribe(topic, 1, cloudCallback);
    ROS_INFO_STREAM("subscribed to " << topic << "; publishing detections on "
        << privateNh.resolveName("ground_cloud") << ", "
        << privateNh.resolveName("obstacle_cloud") << ", "
        << privateNh.resolveName("detection_boxes") << ", "
        << privateNh.resolveName("box_distances")
        << " - view in rviz with Fixed Frame set to the input cloud's frame_id");

    ros::spin();

    delete pointProcessorI;
    return 0;
}
