Project(GlobalOptimizationGraph)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")


set( CMAKE_BUILD_TYPE "Debug" )
#set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)
#set(g2o_PACKAGE_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)

find_package(Boost COMPONENTS serialization wserialization system REQUIRED)
find_package(Glog)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    visualization_msgs
    message_filters

)


include_directories(${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ygz_common_headers
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/
	${PROJECT_SOURCE_DIR/Thirdparty/Sophus/sophus}
	${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}

	)
set (THIRD_PARTY_LIBS
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
	${EIGEN3_LIBS}
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
        ${GLOG_LIBRARIES}
)
add_library(G2OTypesLib G2OTypes.cpp)

add_library(ROS_IO_Manager ROS_IO_Manager.cpp)


add_library(GlobalOptimizationGraph GlobalOptimizationGraph.cpp)
target_link_libraries(GlobalOptimizationGraph
        gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}
	G2OTypesLib
	${THIRD_PARTY_LIBS})

add_executable(GlobalOptimizationGraph_main main.cpp)

target_link_libraries(GlobalOptimizationGraph_main
			G2OTypesLib
			ROS_IO_Manager
			GlobalOptimizationGraph
			${THIRD_PARTY_LIBS}
)





