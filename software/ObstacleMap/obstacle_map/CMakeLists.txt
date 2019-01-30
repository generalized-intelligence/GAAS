cmake_minimum_required(VERSION 2.8.3)
project(stereo_dense_reconstruction)
include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#if(COMPILER_SUPPORTS_CXX11)
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#endif()
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
set(LIBELAS_SRC_DIR src/elas)
set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
#find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)
find_package(  Boost REQUIRED COMPONENTS filesystem )


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

FILE(GLOB LIBELAS_SRC_FILES "src/elas/*.cpp")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${LIBELAS_SRC_DIR}
  ${Boost_INCLUDE_DIRS}
  ${INC_DIR}
  ${Eigen_DIRS}
  ${PCL_INCLUDE_DIRS}
)

rosbuild_add_executable(dense_reconstruction
  src/dense_reconstruction.cpp
  ${LIBELAS_SRC_FILES}
)
target_link_libraries(dense_reconstruction ${OpenCV_LIBS} "-lpopt -lboost_system")

rosbuild_add_executable(match_pointcloud_and_slam_pose
		src/match_pointcloud_and_slam_pose.cpp
		${LIBELAS_SRC_FILES}
		${PCL_LIBRARY_DIRS}
		)
target_link_libraries(match_pointcloud_and_slam_pose 
			${OpenCV_LIBS}
			${PCL_LIBRARY_DIRS}
			 "-lpopt -lboost_system")


