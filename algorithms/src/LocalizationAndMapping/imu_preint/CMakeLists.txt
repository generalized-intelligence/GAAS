cmake_minimum_required(VERSION 2.8.3)
project(imu_preint)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11 -O0)
add_compile_options(-std=c++17 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_BUILD_TYPE RELEASE)
find_package(OpenCV 3.4.5 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost 1.65 COMPONENTS filesystem iostreams timer REQUIRED)
find_package(catkin REQUIRED COMPONENTS
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
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
)
catkin_package()
set(REQUIRED_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GTSAM_LIBRARIES}
        ${Boost_LIBRARIES}
        glog
        gtsam
)
add_executable(offline_calc src/offline_calc.cpp)
target_link_libraries(offline_calc ${REQUIRED_LIBS})

add_executable(imu_preint_node src/imu_preint_node.cpp)
target_link_libraries(imu_preint_node ${REQUIRED_LIBS})




