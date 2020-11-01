cmake_minimum_required(VERSION 2.8)
project(vision_lidar_fusion)

find_package(OpenCV 3.4.5 REQUIRED)
find_package(PCL 1.8 REQUIRED)
#find_package(PCL REQUIRED)
find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
                                cv_bridge
                                image_transport
                                roscpp
                                sensor_msgs
                                geometry_msgs
                                nav_msgs
                                std_msgs
                                octomap_msgs
                                octomap_rviz_plugins
                                visualization_msgs
                                tf)

include_directories(${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)



set(REQUIRED_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)



add_executable(vision_lidar_fusion_node src/vision_lidar_fusion_node.cpp)
target_link_libraries(vision_lidar_fusion_node ${REQUIRED_LIBS})

