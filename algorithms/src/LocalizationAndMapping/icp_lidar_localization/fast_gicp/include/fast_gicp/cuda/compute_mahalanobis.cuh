#ifndef FAST_GICP_CUDA_COMPUTE_MAHALANOBIS_CUH
#define FAST_GICP_CUDA_COMPUTE_MAHALANOBIS_CUH

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thrust/device_vector.h>

#include <fast_gicp/cuda/gaussian_voxelmap.cuh>

namespace fast_gicp {
  namespace cuda {

void compute_mahalanobis(
  const thrust::device_vector<Eigen::Vector3f>& src_points,
  const thrust::device_vector<Eigen::Matrix3f>& src_covs,
  const GaussianVoxelMap& voxelmap,
  const thrust::device_vector<int>& voxel_correspondences,
  const Eigen::Isometry3f& linearized_x,
  thrust::device_vector<Eigen::Matrix3f>& mahalanobis
  );

  }
}  // namespace fast_gicp

#endif