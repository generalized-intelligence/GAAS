cmake_minimum_required(VERSION 2.6)
project(gaas_planner)

set( CMAKE_BUILD_TYPE "Release" )
#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(OpenCV 3.4.5 REQUIRED)
find_package(Glog REQUIRED)
MESSAGE("OPENCV PATH: ${OpenCV_INCLUDE_DIRS}")

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
                    roscpp
                    cv_bridge
                    image_transport
                    sensor_msgs
                    std_msgs
                    message_generation
                    voxblox_ros
            )


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/coin-or
  ${GLOG_INCLUDE_DIRS}
)

set(SOURCES
    src/astar.cpp
    src/hybrid_astar.cpp
    src/utils/path_visulization.cpp
    src/plan_enviroment/enviroment.cpp
    src/path_optimization/cubic_bspline.cpp
    src/path_optimization/bspline_nlp.cpp
)
#FILE(GLOB_RECURSE sourcefiles "src/*.cpp")


add_executable(gaas_planner main.cpp ${SOURCES})

target_link_libraries(
  gaas_planner
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ipopt
  glog
)

#install(TARGETS gaas_planner RUNTIME DESTINATION bin)
