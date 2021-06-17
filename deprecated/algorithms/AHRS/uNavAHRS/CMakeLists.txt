project(AHRS)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${PROJECT_SOURCE_DIR})
add_library(uNavAHRS
                 uNavAHRS.cpp)

add_subdirectory(ros_src)
