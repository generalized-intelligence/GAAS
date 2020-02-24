cmake_minimum_required(VERSION 2.6)
project(gaas_planner)

#set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3.4.5 REQUIRED)
MESSAGE("OPENCV PATH: ${OpenCV_INCLUDE_DIRS}")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  
)

add_executable(gaas_planner main.cpp)

target_link_libraries(
  gaas_planner
  ${OpenCV_LIBS}
)

install(TARGETS gaas_planner RUNTIME DESTINATION bin)
