Project(GlobalOptimizationGraph)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

find_package(OpenCV 3.3.1 REQUIRED)
set(g2o_PACKAGE_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)

find_package(Boost COMPONENTS serialization wserialization system REQUIRED)
find_package(g2o REQUIRED PATH g2o_PACKAGE_PATH)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    visualization_msgs
    message_filters

)


include_directories(${PROJECT_SOURCE_DIR}
	
	)
add_library(G2OTypesLib G2OTypes.cpp)
add_executable(GlobalOptimizationGraph GlobalOptimizationGraph.cpp)

target_link_libraries(GlobalOptimizationGraph
			G2OTypesLib)





