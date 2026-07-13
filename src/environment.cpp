#include "sensors/lidar.h"
#include "city_block.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/************ use simpleHighway to test simulated Lidar data *******/
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    Lidar* lidar_sensor = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcd = lidar_sensor ->scan();
    // renderRays(viewer, lidar_sensor->position, lidar_pcd);
    renderPointCloud(viewer, lidar_pcd, "lidar_pcd");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // on the stack
    // segment plane for road plane isolation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(lidar_pcd, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // clutering on non-road points, to mark objects on road
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

          // add bounding boxes
          if (render_box) {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
          }
          ++clusterId;
    }
}

/*****************************************************************/
// cityBlock() and initCamera() are shared with environment_ros.cpp; see city_block.h
/*****************************************************************/

/********************** main function ************************/
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // uncomment following for simpleHighway function
    // simpleHighway(viewer);
    // cityBlock(viewer);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // }

    // Modified for streming using cityBlock function
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
      {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        // stream
        streamIterator++;
        if(streamIterator == stream.end())
          streamIterator = stream.begin();
        viewer->spinOnce ();
      }
}
