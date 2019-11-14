Project(GlobalOptimizationGraph)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")


set( CMAKE_BUILD_TYPE "Debug" )
#set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(OpenCV 3.4.5 REQUIRED)

find_package(Boost COMPONENTS serialization wserialization system REQUIRED)
find_package(Glog)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    visualization_msgs
    message_filters
    roseus
)


include_directories(${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/ygz_common_headers
	                ${EIGEN3_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GTSAM_INCLUDE_DIR}
		            ${GTSAM_UNSTABLE_INCLUDE_DIR})

set (THIRD_PARTY_LIBS
	${EIGEN3_LIBS}
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}
)

add_library(ROS_IO_Manager ROS_IO_Manager.cpp)

add_library(GlobalOptimizationGraph GlobalOptimizationGraph.cpp)
target_link_libraries(GlobalOptimizationGraph
        gtsam
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
	    ${THIRD_PARTY_LIBS}
		gtsam_unstable)

add_executable(GlobalOptimizationGraph_main main.cpp)
target_link_libraries(GlobalOptimizationGraph_main
			ROS_IO_Manager
			GlobalOptimizationGraph
			${THIRD_PARTY_LIBS}
)

add_executable(test1 test.cpp)
target_link_libraries(test1
			ROS_IO_Manager
			GlobalOptimizationGraph
			${THIRD_PARTY_LIBS}
)



