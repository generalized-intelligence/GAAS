cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(transform_pcl)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(  Boost REQUIRED COMPONENTS filesystem )

find_package(Eigen3  REQUIRED)
find_package(PCL 1.9.0 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
                    roscpp
                    rospy
                    cv_bridge
                    image_transport
                    sensor_msgs
                    std_msgs
                    message_generation
                    pcl_conversions
                    pcl_ros
                    
            )

include_directories(${PCL_INCLUDE_DIRS}
                ${catkin_INCLUDE_DIRS}
                ${Boost_INCLUDE_DIRS}
                ${Eigen_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (transform_cam_to_world transform_cam_to_world.cpp)
target_link_libraries (
            transform_cam_to_world 
                    ${PCL_LIBRARIES}
                    ${OpenCV_LIBS}
                    ${catkin_LIBRARIES}
                    ${Eigen_DIRS}
                    "-lpopt -lboost_system"
)