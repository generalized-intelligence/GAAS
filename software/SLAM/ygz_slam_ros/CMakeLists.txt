cmake_minimum_required( VERSION 2.8 )
project(YGZ-Stereo)

# uncommont this to use release
set( CMAKE_BUILD_TYPE "Release" )
#set(CMAKE_BUILD_TYPE "Debug")

# make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
else()
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}', turn on the compiler optimization")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wall -march=native -O3 -Wno-reorder -pthread -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native -O3 -pthread -fopenmp")
endif()



# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lstdc++fs")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# for tx2
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
# for x86
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -march=native")


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

# thirdparty libs 
find_package(OpenCV REQUIRED)
MESSAGE("OPENCV PATH: ${OpenCV_INCLUDE_DIRS}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Glog REQUIRED )
find_package(Boost COMPONENTS serialization wserialization system REQUIRED)

find_package(G2O REQUIRED)

MESSAGE("G2O PATH: ${G2O_INCLUDE_DIR}")

find_package(PCL REQUIRED)
# octomap
find_package( octomap REQUIRED )
find_package(catkin REQUIRED COMPONENTS
                                cv_bridge
                                image_transport
                                roscpp
                                sensor_msgs
				geometry_msgs
                                nav_msgs
                                std_msgs
                                octomap_msgs
                                octomap_rviz_plugins
                                visualization_msgs
                                tf)
#find_package(CUDA  REQUIRED)
find_package(DBoW3 REQUIRED PATH Thirdparty/DBow3/build)
#find_package(DBoW3)
# to show 'include' in QtCreator
FILE(GLOB_RECURSE INC_ALL "*/*.h")
add_custom_target(headers SOURCES ${INC_ALL})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/Thirdparty/fast/include
#${PROJECT_SOURCE_DIR}/Thirdparty/DBow2/DBoW2
${DBoW3_DIRS}
${OpenCV_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/src
#${PROJECT_SOURCE_DIR}/Thirdparty/DVision/
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/
${G2O_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
)


# thirdparty
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
	${G2O_LIBS}
  	${G2O_CORE_LIBRARY}
 	${G2O_STUFF_LIBRARY}
 	${G2O_TYPES_SLAM2D}
	${G2O_TYPES_SLAM3D}
 	${G2O_SOLVER_CHOLMOD}
	${G2O_SOLVER_CSPARSE}
	${G2O_SOLVER_CSPARSE_EXTENSION}
        #${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/lib/libDBo.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/build/src/libDBoW3.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/build/libloopclosingmanager.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/fast/build/libfast.so
        #${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${GLOG_LIBRARY}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${DBoW3_LIBRARIES}
)

# subdirectories
include_directories(
        util/include
        cv/include
        common/include
        backend/include
        system/include
	serialization/include
)

add_subdirectory(util)

add_subdirectory(common)

add_subdirectory(cv)

add_subdirectory(backend)

add_subdirectory(system)

set(YGZ_LIBS ygz-common ygz-cv ygz-util ygz-backend ygz-system )

# build examples
add_subdirectory( examples )
