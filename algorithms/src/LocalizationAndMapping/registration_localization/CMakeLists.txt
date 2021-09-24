cmake_minimum_required(VERSION 2.8.3)
project(registration_localization)

add_compile_options(-std=c++17 -O3)
set(CMAKE_BUILD_TYPE RELEASE)
#add_compile_options(-std=c++17 -O0)
#set(CMAKE_BUILD_TYPE DEBUG)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    find_package(OpenCV 3.4.5 REQUIRED)
else()
    set (OpenCV_DIR /home/gi/nvme/opencv-3.4.5/build)
endif()

find_package(PCL 1.8 REQUIRED)
#find_package(PCL REQUIRED)
find_package(Glog REQUIRED)


find_package(CUDA) #if you wanna use cpu in a platform with cuda, disable this line.



find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  visualization_msgs
)
set(REQUIRED_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        glog
)
catkin_package()
#======================================================================================================
#add fast_gicp

option(BUILD_apps "Build application programs" ON)


if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
else()
  add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED)
set(Eigen3_DIR fast_gicp/thirdparty/Eigen/)#find_package(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR ${Eigen3_DIR})
if (CUDA_FOUND)
    add_definitions(-D CUDA_FOUND)
    MESSAGE("To support cuda: update your eigen to eigen3.3.9;sudo make install&&sudo ldconfig -v")
    list(APPEND NDT_ACCELERATION_PACKAGE ndt_gpu_lib)
    set (CMAKE_CUDA_CREATE_SHARED_LIBRARY false)
endif ()


#find_package(catkin)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


if(CUDA_FOUND)
#  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

###################################
## catkin specific configuration ##
###################################
#if(catkin_FOUND)
#  catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES fast_gicp
#  )
#endif()

###########
## Build ##
###########

add_executable(map_publisher src/map_publisher.cpp)
target_link_libraries(map_publisher ${REQUIRED_LIBS})


add_library(fast_gicp SHARED
  fast_gicp/src/fast_gicp/gicp/lsq_registration.cpp
  fast_gicp/src/fast_gicp/gicp/fast_gicp.cpp
  fast_gicp/src/fast_gicp/gicp/fast_gicp_st.cpp
  fast_gicp/src/fast_gicp/gicp/fast_vgicp.cpp
)
target_link_libraries(fast_gicp
  ${PCL_LIBRARIES}
)
target_include_directories(fast_gicp PUBLIC
  fast_gicp/include
  ${PCL_INCLUDE_DIRS}
  fast_gicp/thirdparty/Eigen
)



### CUDA ###

set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
add_definitions(-DUSE_VGICP_CUDA)

cuda_add_library(fast_vgicp_cuda SHARED
    fast_gicp/src/fast_gicp/cuda/fast_vgicp_cuda.cu
    fast_gicp/src/fast_gicp/cuda/brute_force_knn.cu
    fast_gicp/src/fast_gicp/cuda/covariance_estimation.cu
    fast_gicp/src/fast_gicp/cuda/covariance_estimation_rbf.cu
    fast_gicp/src/fast_gicp/cuda/covariance_regularization.cu
    fast_gicp/src/fast_gicp/cuda/gaussian_voxelmap.cu
    fast_gicp/src/fast_gicp/cuda/find_voxel_correspondences.cu
    fast_gicp/src/fast_gicp/cuda/compute_derivatives.cu
    fast_gicp/src/fast_gicp/cuda/compute_mahalanobis.cu
    fast_gicp/src/fast_gicp/cuda/ndt_cuda.cu
    fast_gicp/src/fast_gicp/cuda/ndt_compute_derivatives.cu
    )
target_include_directories(fast_vgicp_cuda PRIVATE
    fast_gicp/include
    fast_gicp/thirdparty/Eigen
    fast_gicp/thirdparty/nvbio
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    )
target_link_libraries(fast_vgicp_cuda
    ${catkin_LIBRARIES}
    )
cuda_add_cublas_to_target(fast_vgicp_cuda)

# add vgicp_cuda to libfast_gicp
target_sources(fast_gicp PRIVATE
    fast_gicp/src/fast_gicp/gicp/fast_vgicp_cuda.cpp
    fast_gicp/src/fast_gicp/ndt/ndt_cuda.cpp
    )
target_link_libraries(fast_gicp
    fast_vgicp_cuda
    )
add_dependencies(fast_gicp fast_vgicp_cuda)
if(catkin_FOUND)
    install(TARGETS fast_vgicp_cuda
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

#======================================================================================================
#gicp finished


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_lidar_fusion
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs tf visualization_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vision_lidar_fusion.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide





#if(CUDA_FOUND)
#else()
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        fast_gicp/include/
)
##icp libs


##icp libs end

message("Eigen3 dir:${Eigen3_DIR}")
message("Eigen3 include dir:${EIGEN3_INCLUDE_DIRS}")
message("Eigen include dir:${EIGEN_INCLUDE_DIRS}")

## ndt libs
if(CUDA_FOUND)
    add_definitions(-D CUDA_FOUND)
#    find_package(Eigen3 QUIET)
    set(Eigen3_DIR fast_gicp/thirdparty/Eigen/)#find_package(Eigen3 REQUIRED)
    set(EIGEN_INCLUDE_DIRS ${EIGEN3_DIR})
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_DIR})

#    if (NOT EIGEN3_FOUND)
#        # Fallback to cmake_modules
#        find_package(cmake_modules REQUIRED)
#        find_package(Eigen REQUIRED)
#        set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
#        set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
#        # Possibly map additional variables to the EIGEN3_ prefix.
#    else ()
#        set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#    endif ()

    set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")
    if (NOT DEFINED CUDA_CAPABILITY_VERSION_CHECKER)
        set(CUDA_CAPABILITY_VERSION_CHECKER
                "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker")
    endif ()

    execute_process(COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
            OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE)

#    if ("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
#        set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
#        MESSAGE( "CUDA_ARCH matched: ${CUDA_ARCH}")
#    else ()
#        set(CUDA_ARCH "sm_52")
#        MESSAGE( "CUDA_ARCH set to: ${CUDA_ARCH} by default")
#        MESSAGE( "CUDA_CAPABILITY_VERSION: ${CUDA_CAPABILITY_VERSION}")
#    endif ()
    set(processor_type "${CMAKE_SYSTEM_PROCESSOR}")
    message("Processor type: ${processor_type}")
    if("${processor_type}" MATCHES "aarch64") # Jetson AGX Xavier
        set(CUDA_ARCH "sm_72")
    else() # x86/other
        set(CUDA_ARCH "sm_61")
    endif()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++14;--ptxas-options=-v)

    set(SUBSYS_DESC "Point cloud ndt gpu library")

#    catkin_package(
#            DEPENDS PCL                                #Non-catkin CMake projects
#            INCLUDE_DIRS ndt_gpu/ndt_gpu_include                        #The exported include paths
#            LIBRARIES ndt_gpu_lib                           #The exported libraries from the project
#    )

    set(ndt_gpu_srcs
            ./ndt_gpu/ndt_gpu_src/VoxelGrid.cu
            ./ndt_gpu/ndt_gpu_src/MatrixDevice.cu
            ./ndt_gpu/ndt_gpu_src/SymmetricEigenSolver.cu
            ./ndt_gpu/ndt_gpu_src/MatrixHost.cu
            ./ndt_gpu/ndt_gpu_src/Registration.cu
            ./ndt_gpu/ndt_gpu_src/NormalDistributionsTransform.cu
       )
    set(incs
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/Registration.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/VoxelGrid.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/NormalDistributionsTransform.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/SymmetricEigenSolver.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/MatrixDevice.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/Matrix.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/debug.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/MatrixHost.h
            ./ndt_gpu/ndt_gpu_include/ndt_gpu/common.h
       )

    include_directories(
            fast_gicp/thirdparty/Eigen
            ${PCL_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            ${CUDA_INCLUDE_DIRS}
            ndt_gpu/ndt_gpu_include
#            fast_gicp/thirdparty/nvbio
#            ${catkin_INCLUDE_DIRS}
#
#            ${EIGEN3_INCLUDE_DIRS}
    )
    cuda_add_library(ndt_gpu_lib ${ndt_gpu_srcs} ${incs})
    target_link_libraries(ndt_gpu_lib
            ${CUDA_LIBRARIES}
            #${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${PCL_LIBRARIES}
            ${catkin_LIBRARIES}
            )

##    add_executable(ndt_matching_node src/ndt_matching.cpp)
##    target_link_libraries(ndt_matching_node ${REQUIRED_LIBS} ndt_gpu_lib
##            ${CUDA_LIBRARIES}
##            #${CUDA_CUBLAS_LIBRARIES}
##            ${CUDA_curand_LIBRARY}
##            )

#else()
    set(ndt_cpu_srcs
        ndt_cpu/ndt_cpu_src/NormalDistributionsTransform.cpp
        ndt_cpu/ndt_cpu_src/Registration.cpp
        ndt_cpu/ndt_cpu_src/VoxelGrid.cpp
        ndt_cpu/ndt_cpu_src/Octree.cpp
        )

    set(incs
        ndt_cpu/ndt_cpu_include/ndt_cpu/debug.h
        ndt_cpu/ndt_cpu_include/ndt_cpu/NormalDistributionsTransform.h
        ndt_cpu/ndt_cpu_include/ndt_cpu/Registration.h
        ndt_cpu/ndt_cpu_include/ndt_cpu/SymmetricEigenSolver.h
        ndt_cpu/ndt_cpu_include/ndt_cpu/VoxelGrid.h
        ndt_cpu/ndt_cpu_include/ndt_cpu/Octree.h
        )

    include_directories(
            fast_gicp/thirdparty/Eigen
            ${PCL_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            ndt_cpu/ndt_cpu_include
    )

    add_library(ndt_cpu_lib ${ndt_cpu_srcs})
    set_target_properties(ndt_cpu_lib PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(ndt_cpu_lib
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        )

#    add_executable(ndt_matching_node src/ndt_matching.cpp)
#    target_link_libraries(ndt_matching_node ${REQUIRED_LIBS} ndt_cpu_lib)
endif()


## ndt libs end.





add_executable(map_manager_test src/map_manager_test.cpp)
target_link_libraries(map_manager_test ${REQUIRED_LIBS})
add_executable(registration_localization_node src/registration_localization_node.cpp)
target_link_libraries(registration_localization_node ${REQUIRED_LIBS} fast_gicp ndt_cpu_lib ndt_gpu_lib)

#map publisher
#add_executable(map_publisher src/map_publisher.cpp)
#target_link_libraries(map_publisher ${REQUIRED_LIBS})


#add_executable(icp_matching_node src/icp_matching.cpp)
#target_link_libraries(icp_matching_node ${REQUIRED_LIBS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision_lidar_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

