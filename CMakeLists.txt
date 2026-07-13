cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lidar_obstacle_detection)

add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs pcl_conversions
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# Offline .pcd-file playback (PCL viewer, no ROS dependency).
add_executable(environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries(environment ${PCL_LIBRARIES})

# Live ROS1 node: subscribes to a PointCloud2 topic (e.g. a Velodyne VLP-16),
# runs the same filter->segment->cluster->box pipeline, publishes results for
# RViz. Used for close-range (<2m) obstacle detection.
add_executable(environment_ros src/environment_ros.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries(environment_ros ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(environment_ros ${catkin_EXPORTED_TARGETS})

# Range-gate node: splits one incoming PointCloud2 stream into a "near"
# (<range_threshold, default 2m) and "far" (>=range_threshold) stream, so the
# close-range clustering pipeline and the mid/long-range PointPillars
# detector each only ever see the points meant for them.
add_executable(range_gate src/range_gate.cpp)
target_link_libraries(range_gate ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(range_gate ${catkin_EXPORTED_TARGETS})

install(TARGETS environment environment_ros range_gate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
