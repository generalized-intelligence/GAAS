cmake_minimum_required(VERSION 2.8.3)
project(hud)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11 -O0)
add_compile_options(-std=c++17 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_BUILD_TYPE RELEASE)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    find_package(OpenCV 3.4.5 REQUIRED)
else()
    set (OpenCV_DIR /home/gi/nvme/opencv-3.4.5/build)
endif()


find_package(PCL 1.8 REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gaas_msgs
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rosbag
  sensor_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  visualization_msgs
)
catkin_package()
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

set(REQUIRED_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GTSAM_LIBRARIES}
        glog
)
add_executable(HUD_node src/HUD_node.cpp)
target_link_libraries(HUD_node ${REQUIRED_LIBS})




