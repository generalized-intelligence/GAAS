cmake_minimum_required(VERSION 2.8.3)
project(json_request_response_lib)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17 -O0)
add_compile_options(-std=c++17 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_BUILD_TYPE RELEASE)
#find_package(OpenCV 3.4.5 REQUIRED)
find_package(Glog REQUIRED)
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
)

set(REQUIRED_LIBS
        ${GLOG_LIBRARY}
        ${catkin_LIBRARIES}
        glog
)
add_executable(test_json_client src/test_json_client.cpp)
target_link_libraries(test_json_client ${REQUIRED_LIBS})

add_executable(test_json_server src/test_json_server.cpp)
target_link_libraries(test_json_server ${REQUIRED_LIBS})




