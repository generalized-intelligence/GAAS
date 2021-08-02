cmake_minimum_required(VERSION 2.8.3)
project (test_ground_point_removal)
add_compile_options(-std=c++11 -O3)

set(CMAKE_BUILD_TYPE RELEASE)

find_package(PCL 1.8 REQUIRED)
find_package(Glog REQUIRED)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)
set(REQUIRED_LIBS
        ${PCL_LIBRARIES}
        glog
)

add_executable(ground_point_removal_test src/ground_point_removal_test.cpp)
target_link_libraries(ground_point_removal_test ${REQUIRED_LIBS})
