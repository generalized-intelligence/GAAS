#ifndef FAST_GICP_CUDA_COVARIANCE_REGULARIZATION_CUH
#define FAST_GICP_CUDA_COVARIANCE_REGULARIZATION_CUH

#include <Eigen/Core>
#include <thrust/device_vector.h>
#include <fast_gicp/gicp/gicp_settings.hpp>

namespace fast_gicp {
namespace cuda {

void covariance_regularization(thrust::device_vector<Eigen::Vector3f>& means, thrust::device_vector<Eigen::Matrix3f>& covs, RegularizationMethod method);

}  // namespace cuda
}  // namespace fast_gicp

#endif