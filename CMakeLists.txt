cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})

# Offline batch evaluation entry point: reads a directory of .pcd files and
# writes one detection JSON per frame, named after the input frame - no ROS,
# no PCL viewer, no rosbag replay/timestamp-sync involved.
add_executable (environment_offline_eval src/environment_offline_eval.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment_offline_eval ${PCL_LIBRARIES} boost_filesystem boost_system)

# Live ROS1/VLP-16 node - built only when ROS (roscpp) is discoverable via
# pkg-config, i.e. after `source /opt/ros/<distro>/setup.bash`. Uses
# pkg-config rather than find_package(catkin ...) so this stays a plain
# CMake build with no catkin workspace required.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ROSCPP QUIET roscpp sensor_msgs pcl_conversions visualization_msgs)
endif()

if(ROSCPP_FOUND)
  include_directories(${ROSCPP_INCLUDE_DIRS})
  link_directories(${ROSCPP_LIBRARY_DIRS})

  add_executable (environment_ros src/environment_ros.cpp src/render/render.cpp src/processPointClouds.cpp)
  target_compile_options(environment_ros PRIVATE ${ROSCPP_CFLAGS_OTHER})
  target_link_libraries (environment_ros ${PCL_LIBRARIES} ${ROSCPP_LIBRARIES})
else()
  message(STATUS "ROS (roscpp) not found via pkg-config - skipping environment_ros target. Run 'source /opt/ros/<distro>/setup.bash' before cmake to enable it.")
endif()
