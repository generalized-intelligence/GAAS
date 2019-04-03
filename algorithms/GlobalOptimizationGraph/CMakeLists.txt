Project(GlobalOptimizationGraph)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 3.3.1 REQUIRED)
set(g2o_PACKAGE_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)

find_package(Boost COMPONENTS serialization wserialization system REQUIRED)
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

	)
set (THIRD_PARTY_LIBS
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
	${EIGEN3_LIBS}
)
add_library(G2OTypesLib G2OTypes.cpp)

add_library(ROS_IO_Manager ROS_IO_Manager.cpp)


add_library(GlobalOptimizationGraph GlobalOptimizationGraph.cpp)

add_executable(GlobalOptimizationGraph_main main.cpp)

target_link_libraries(GlobalOptimizationGraph_main
			G2OTypesLib
			ROS_IO_Manager
			GlobalOptimizationGraph
			${THIRD_PARTY_LIBS}
)





