#include <fast_gicp/cuda/ndt_compute_derivatives.cuh>

#include <thrust/transform_reduce.h>

namespace fast_gicp {
namespace cuda {

namespace {

__host__ __device__ float huber(float k, float x) {
  float abs_x = fabsf(x);
  return abs_x <= k ? 1.0 : k / abs_x;
}

__host__ __device__ float cauchy(float k, float x) {
  float k_sq = k * k;
  return k_sq / (k_sq + x * x);
}

// skew symmetric matrix
__host__ __device__ Eigen::Matrix3f skew_symmetric(const Eigen::Vector3f& x) {
  Eigen::Matrix3f skew = Eigen::Matrix3f::Zero();
  skew(0, 1) = -x[2];
  skew(0, 2) = x[1];
  skew(1, 0) = x[2];
  skew(1, 2) = -x[0];
  skew(2, 0) = -x[1];
  skew(2, 1) = x[0];

  return skew;
}

struct p2d_ndt_compute_derivatives_kernel {
  p2d_ndt_compute_derivatives_kernel(
    const GaussianVoxelMap& target_voxelmap,
    const thrust::device_vector<Eigen::Vector3f>& source_points,
    const thrust::device_ptr<const Eigen::Isometry3f>& x_eval_ptr,
    const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr)
  : trans_eval_ptr(x_eval_ptr),
    trans_ptr(x_ptr),
    src_means_ptr(source_points.data()),
    voxelmap_info_ptr(target_voxelmap.voxelmap_info_ptr.data()),
    voxel_num_points_ptr(target_voxelmap.num_points.data()),
    voxel_means_ptr(target_voxelmap.voxel_means.data()),
    voxel_covs_ptr(target_voxelmap.voxel_covs.data()) {}

  // Here, we use simple point-to-distribution MLE distance equivalent to [Biber, IROS2003]
  // Because this formulation can be more sensitive to outliers compared to another formulation based on
  // Gaussian + uniform distribution [Magnusson, 2009], we use some robust kernels to filter out outliers
  __host__ __device__ thrust::tuple<float, Eigen::Matrix<float, 6, 6>, Eigen::Matrix<float, 6, 1>> operator()(const thrust::pair<int, int>& correspondence) const {
    const Eigen::Vector3f& mean_A = thrust::raw_pointer_cast(src_means_ptr)[correspondence.first];

    if (correspondence.second < 0) {
      return thrust::make_tuple(0.0f, Eigen::Matrix<float, 6, 6>::Zero().eval(), Eigen::Matrix<float, 6, 1>::Zero().eval());
    }

    int num_points = thrust::raw_pointer_cast(voxel_num_points_ptr)[correspondence.second];
    const Eigen::Vector3f& mean_B = thrust::raw_pointer_cast(voxel_means_ptr)[correspondence.second];
    const Eigen::Matrix3f& cov_B = thrust::raw_pointer_cast(voxel_covs_ptr)[correspondence.second];

    if (num_points <= 6) {
      return thrust::make_tuple(0.0f, Eigen::Matrix<float, 6, 6>::Zero().eval(), Eigen::Matrix<float, 6, 1>::Zero().eval());
    }

    const auto& trans_eval = *thrust::raw_pointer_cast(trans_eval_ptr);
    const auto& trans = *thrust::raw_pointer_cast(trans_ptr);

    Eigen::Matrix3f R_eval = trans_eval.linear();
    Eigen::Matrix3f R = trans.linear();
    Eigen::Vector3f t = trans.translation();

    const Eigen::Vector3f transed_mean_A = R * mean_A + t;

    Eigen::Matrix3f RCR_inv = cov_B.inverse();

    Eigen::Vector3f error = mean_B - transed_mean_A;

    float w = cauchy(thrust::raw_pointer_cast(voxelmap_info_ptr)->voxel_resolution, error.norm());
    float err = w * error.transpose() * RCR_inv * error;

    Eigen::Matrix<float, 3, 6> dtdx0;
    dtdx0.block<3, 3>(0, 0) = skew_symmetric(transed_mean_A);
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3f::Identity();

    Eigen::Matrix<float, 3, 6> J = dtdx0;

    Eigen::Matrix<float, 6, 6> H = w * J.transpose() * RCR_inv * J;
    Eigen::Matrix<float, 6, 1> b = w * J.transpose() * RCR_inv * error;

    return thrust::make_tuple(err, H, b);
  }

  thrust::device_ptr<const Eigen::Isometry3f> trans_eval_ptr;
  thrust::device_ptr<const Eigen::Isometry3f> trans_ptr;

  thrust::device_ptr<const Eigen::Vector3f> src_means_ptr;

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const int> voxel_num_points_ptr;
  thrust::device_ptr<const Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<const Eigen::Matrix3f> voxel_covs_ptr;
};

struct d2d_ndt_compute_derivatives_kernel {
  d2d_ndt_compute_derivatives_kernel(
    const GaussianVoxelMap& target_voxelmap,
    const GaussianVoxelMap& source_voxelmap,
    const thrust::device_ptr<const Eigen::Isometry3f>& x_eval_ptr,
    const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr)
  : trans_eval_ptr(x_eval_ptr),
    trans_ptr(x_ptr),
    src_means_ptr(source_voxelmap.voxel_means.data()),
    src_covs_ptr(source_voxelmap.voxel_covs.data()),
    voxelmap_info_ptr(target_voxelmap.voxelmap_info_ptr.data()),
    voxel_num_points_ptr(target_voxelmap.num_points.data()),
    voxel_means_ptr(target_voxelmap.voxel_means.data()),
    voxel_covs_ptr(target_voxelmap.voxel_covs.data()) {}

  // calculate derivatives
  __host__ __device__ thrust::tuple<float, Eigen::Matrix<float, 6, 6>, Eigen::Matrix<float, 6, 1>> operator()(const thrust::pair<int, int>& correspondence) const {
    const Eigen::Vector3f& mean_A = thrust::raw_pointer_cast(src_means_ptr)[correspondence.first];
    const Eigen::Matrix3f& cov_A = thrust::raw_pointer_cast(src_covs_ptr)[correspondence.first];

    if (correspondence.second < 0) {
      return thrust::make_tuple(0.0f, Eigen::Matrix<float, 6, 6>::Zero().eval(), Eigen::Matrix<float, 6, 1>::Zero().eval());
    }

    int num_points = thrust::raw_pointer_cast(voxel_num_points_ptr)[correspondence.second];
    const Eigen::Vector3f& mean_B = thrust::raw_pointer_cast(voxel_means_ptr)[correspondence.second];
    const Eigen::Matrix3f& cov_B = thrust::raw_pointer_cast(voxel_covs_ptr)[correspondence.second];

    if (num_points <= 6) {
      return thrust::make_tuple(0.0f, Eigen::Matrix<float, 6, 6>::Zero().eval(), Eigen::Matrix<float, 6, 1>::Zero().eval());
    }

    const auto& trans_eval = *thrust::raw_pointer_cast(trans_eval_ptr);
    const auto& trans = *thrust::raw_pointer_cast(trans_ptr);

    Eigen::Matrix3f R_eval = trans_eval.linear();
    Eigen::Matrix3f R = trans.linear();
    Eigen::Vector3f t = trans.translation();

    const Eigen::Vector3f transed_mean_A = R * mean_A + t;

    Eigen::Matrix3f RCR = R_eval * cov_A * R_eval.transpose();
    Eigen::Matrix3f RCR_inv = (cov_B + RCR).inverse();

    Eigen::Vector3f error = mean_B - transed_mean_A;

    float w = cauchy(thrust::raw_pointer_cast(voxelmap_info_ptr)->voxel_resolution, error.norm());
    float err = w * error.transpose() * RCR_inv * error;

    Eigen::Matrix<float, 3, 6> dtdx0;
    dtdx0.block<3, 3>(0, 0) = skew_symmetric(transed_mean_A);
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3f::Identity();

    Eigen::Matrix<float, 3, 6> J = dtdx0;

    Eigen::Matrix<float, 6, 6> H = w * J.transpose() * RCR_inv * J;
    Eigen::Matrix<float, 6, 1> b = w * J.transpose() * RCR_inv * error;

    return thrust::make_tuple(err, H, b);
  }

  thrust::device_ptr<const Eigen::Isometry3f> trans_eval_ptr;
  thrust::device_ptr<const Eigen::Isometry3f> trans_ptr;

  thrust::device_ptr<const Eigen::Vector3f> src_means_ptr;
  thrust::device_ptr<const Eigen::Matrix3f> src_covs_ptr;

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const int> voxel_num_points_ptr;
  thrust::device_ptr<const Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<const Eigen::Matrix3f> voxel_covs_ptr;
};

struct sum_errors_kernel {
  using Tuple = thrust::tuple<float, Eigen::Matrix<float, 6, 6>, Eigen::Matrix<float, 6, 1>>;

  __host__ __device__ Tuple operator()(const Tuple& lhs, const Tuple& rhs) {
    return thrust::make_tuple(thrust::get<0>(lhs) + thrust::get<0>(rhs), thrust::get<1>(lhs) + thrust::get<1>(rhs), thrust::get<2>(lhs) + thrust::get<2>(rhs));
  }
};

}  // namespace

double p2d_ndt_compute_derivatives(
  const GaussianVoxelMap& target_voxelmap,
  const thrust::device_vector<Eigen::Vector3f>& source_points,
  const thrust::device_vector<thrust::pair<int, int>>& correspondences,
  const thrust::device_ptr<const Eigen::Isometry3f>& linearized_x_ptr,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr,
  Eigen::Matrix<double, 6, 6>* H,
  Eigen::Matrix<double, 6, 1>* b) {
  auto sum_errors = thrust::transform_reduce(
    correspondences.begin(),
    correspondences.end(),
    p2d_ndt_compute_derivatives_kernel(target_voxelmap, source_points, linearized_x_ptr, x_ptr),
    thrust::make_tuple(0.0f, Eigen::Matrix<float, 6, 6>::Zero().eval(), Eigen::Matrix<float, 6, 1>::Zero().eval()),
    sum_errors_kernel());

  if (H && b) {
    *H = thrust::get<1>(sum_errors).cast<double>();
    *b = thrust::get<2>(sum_errors).cast<double>();
  }

  return thrust::get<0>(sum_errors);
}

double d2d_ndt_compute_derivatives(
  const GaussianVoxelMap& target_voxelmap,
  const GaussianVoxelMap& source_voxelmap,
  const thrust::device_vector<thrust::pair<int, int>>& correspondences,
  const thrust::device_ptr<const Eigen::Isometry3f>& linearized_x_ptr,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr,
  Eigen::Matrix<double, 6, 6>* H,
  Eigen::Matrix<double, 6, 1>* b) {
  auto sum_errors = thrust::transform_reduce(
    correspondences.begin(),
    correspondences.end(),
    d2d_ndt_compute_derivatives_kernel(target_voxelmap, source_voxelmap, linearized_x_ptr, x_ptr),
    thrust::make_tuple(0.0f, Eigen::Matrix<float, 6, 6>::Zero().eval(), Eigen::Matrix<float, 6, 1>::Zero().eval()),
    sum_errors_kernel());

  if (H && b) {
    *H = thrust::get<1>(sum_errors).cast<double>();
    *b = thrust::get<2>(sum_errors).cast<double>();
  }

  return thrust::get<0>(sum_errors);
}

}  // namespace cuda
}  // namespace fast_gicp
