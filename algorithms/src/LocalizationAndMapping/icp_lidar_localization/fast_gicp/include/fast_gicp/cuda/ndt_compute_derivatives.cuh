#ifndef FAST_GICP_CUDA_NDT_COMPUTE_DERIVATIVES_CUH
#define FAST_GICP_CUDA_NDT_COMPUTE_DERIVATIVES_CUH

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thrust/device_vector.h>

#include <fast_gicp/cuda/gaussian_voxelmap.cuh>

namespace fast_gicp {
namespace cuda {

double p2d_ndt_compute_derivatives(
  const GaussianVoxelMap& target_voxelmap,
  const thrust::device_vector<Eigen::Vector3f>& source_points,
  const thrust::device_vector<thrust::pair<int, int>>& correspondences,
  const thrust::device_ptr<const Eigen::Isometry3f>& linearized_x_ptr,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr,
  Eigen::Matrix<double, 6, 6>* H,
  Eigen::Matrix<double, 6, 1>* b);

double d2d_ndt_compute_derivatives(
  const GaussianVoxelMap& target_voxelmap,
  const GaussianVoxelMap& source_voxelmap,
  const thrust::device_vector<thrust::pair<int, int>>& correspondences,
  const thrust::device_ptr<const Eigen::Isometry3f>& linearized_x_ptr,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr,
  Eigen::Matrix<double, 6, 6>* H,
  Eigen::Matrix<double, 6, 1>* b);

}  // namespace cuda
}  // namespace fast_gicp

#endif