Project(GlobalOptimizationGraph)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

find_package(OpenCV 3.3.1 REQUIRED)
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





