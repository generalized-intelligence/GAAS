cmake_minimum_required(VERSION 2.6)
project(gaas_planner)

set( CMAKE_BUILD_TYPE "Release" )
#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 11)

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
            )


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

set(SOURCES
    src/astar.cpp
    src/hybrid_astar.cpp
    src/utils/path_visulization.cpp
)
#FILE(GLOB_RECURSE sourcefiles "src/*.cpp")

add_executable(gaas_planner main.cpp ${SOURCES})

target_link_libraries(
  gaas_planner
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  glog
)

install(TARGETS gaas_planner RUNTIME DESTINATION bin)
