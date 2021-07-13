#ifndef FAST_GICP_CUDA_COVARIANCE_ESTIMATION_CUH
#define FAST_GICP_CUDA_COVARIANCE_ESTIMATION_CUH

#include <Eigen/Core>
#include <thrust/device_vector.h>
#include <fast_gicp/gicp/gicp_settings.hpp>

namespace fast_gicp {
namespace cuda {

void covariance_estimation(const thrust::device_vector<Eigen::Vector3f>& points, int k, const thrust::device_vector<int>& k_neighbors, thrust::device_vector<Eigen::Matrix3f>& covariances);

void covariance_estimation_rbf(const thrust::device_vector<Eigen::Vector3f>& points, double kernel_width, double max_dist, thrust::device_vector<Eigen::Matrix3f>& covariances);
}
  }  // namespace fast_gicp

#endif