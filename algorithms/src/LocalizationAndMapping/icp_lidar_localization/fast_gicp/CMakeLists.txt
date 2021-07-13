cmake_minimum_required(VERSION 3.10.0)
project(fast_gicp)

option(BUILD_apps "Build application programs" ON)


if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
else()
  add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

#find_package(catkin)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})

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
  ${EIGEN3_INCLUDE_DIR}
)


### CUDA ###
if(CUDA_FOUND)
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
else()

endif()



if(catkin_FOUND)
  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  
  install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp")
endif()
