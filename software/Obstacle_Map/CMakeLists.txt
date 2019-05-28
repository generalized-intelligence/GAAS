cmake_minimum_required(VERSION 2.8.3)
project(stereo_dense_reconstruction)


# uncommont this to use release
#set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_BUILD_TYPE "Debug")


include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#if(COMPILER_SUPPORTS_CXX11)
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#endif()
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (CMAKE_CXX_STANDARD 14)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(Eigen3  REQUIRED)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED)
set(INC_DIR ./include)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
#find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)
find_package(  Boost REQUIRED COMPONENTS filesystem )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${INC_DIR}
  ${Eigen_DIRS}
  ${PCL_INCLUDE_DIRS}
)





add_library(pcl_helper 
	   src/pcl_helper.cpp)


target_link_libraries(pcl_helper
		     ${PCL_LIBRARIES})


rosbuild_add_executable(dense_reconstruction
  src/dense_reconstruction.cpp
)

target_link_libraries(dense_reconstruction
		     ${OpenCV_LIBS}
		     "-lpopt -lboost_system"
		      pcl_helper)

rosbuild_add_executable(match_pointcloud_and_slam_pose
		src/match_pointcloud_and_slam_pose.cpp
		)

target_link_libraries(match_pointcloud_and_slam_pose 
			${OpenCV_LIBS}
			${PCL_LIBRARIES}
			 "-lpopt -lboost_system"
			pcl_helper)

rosbuild_add_executable(match_pointcloud_and_slam_pose2
		src/match_pointcloud_and_slam_pose2.cpp
		)

target_link_libraries(match_pointcloud_and_slam_pose2
			${OpenCV_LIBS}
			${PCL_LIBRARIES}
			 "-lpopt -lboost_system"
			pcl_helper)

#rosbuild_add_executable(rgbd_camera_to_pointcloud
#		src/rgbd_camera_to_pointcloud.cpp
#		)
#
#target_link_libraries(rgbd_camera_to_pointcloud
#			${OpenCV_LIBS}
#			${PCL_LIBRARIES}
#			 "-lpopt -lboost_system"
#			pcl_helper)


