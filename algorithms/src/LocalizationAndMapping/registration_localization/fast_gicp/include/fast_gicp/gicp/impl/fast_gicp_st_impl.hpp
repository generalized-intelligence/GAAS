#ifndef FAST_GICP_FAST_GICP_ST_IMPL_HPP
#define FAST_GICP_FAST_GICP_ST_IMPL_HPP

#include <fast_gicp/so3/so3.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
FastGICPSingleThread<PointSource, PointTarget>::FastGICPSingleThread() : FastGICP<PointSource, PointTarget>() {
  this->reg_name_ = "FastGICPSingleThread";
  this->num_threads_ = 1;
}

template <typename PointSource, typename PointTarget>
FastGICPSingleThread<PointSource, PointTarget>::~FastGICPSingleThread() {}

template <typename PointSource, typename PointTarget>
void FastGICPSingleThread<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  anchors_.clear();
  FastGICP<PointSource, PointTarget>::computeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget>
void FastGICPSingleThread<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& x) {
  assert(source_covs_.size() == input_->size());
  assert(target_covs_.size() == target_->size());

  Eigen::Isometry3f trans = x.template cast<float>();

  bool is_first = anchors_.empty();

  correspondences_.resize(input_->size());
  sq_distances_.resize(input_->size());
  second_sq_distances_.resize(input_->size());
  anchors_.resize(input_->size());
  mahalanobis_.resize(input_->size());

  std::vector<int> k_indices;
  std::vector<float> k_sq_dists;

  for (int i = 0; i < input_->size(); i++) {
    PointTarget pt;
    pt.getVector4fMap() = trans * input_->at(i).getVector4fMap();

    if (!is_first) {
      double d = (pt.getVector4fMap() - anchors_[i]).norm();
      double max_first = std::sqrt(sq_distances_[i]) + d;
      double min_second = std::sqrt(second_sq_distances_[i]) - d;

      if (max_first < min_second) {
        continue;
      }
    }

    target_kdtree_->nearestKSearch(pt, 2, k_indices, k_sq_dists);

    correspondences_[i] = k_sq_dists[0] < this->corr_dist_threshold_ * this->corr_dist_threshold_ ? k_indices[0] : -1;
    sq_distances_[i] = k_sq_dists[0];
    second_sq_distances_[i] = k_sq_dists[1];
    anchors_[i] = pt.getVector4fMap();

    if (correspondences_[i] < 0) {
      continue;
    }

    const int target_index = correspondences_[i];
    const auto& cov_A = source_covs_[i];
    const auto& cov_B = target_covs_[target_index];

    Eigen::Matrix4d RCR = cov_B + x.matrix() * cov_A * x.matrix().transpose();
    RCR(3, 3) = 1.0;

    mahalanobis_[i] = RCR.inverse();
    mahalanobis_[i](3, 3) = 0.0;
  }
}

template <typename PointSource, typename PointTarget>
double FastGICPSingleThread<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  if (H && b) {
    update_correspondences(trans);
    H->setZero();
    b->setZero();
  }

  double sum_errors = 0.0;
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[i];

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    const auto& cov_B = target_covs_[target_index];

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    (*H) += jlossexp.transpose() * mahalanobis_[i] * jlossexp;
    (*b) += jlossexp.transpose() * mahalanobis_[i] * error;
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
double FastGICPSingleThread<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  return linearize(trans, nullptr, nullptr);
}

}  // namespace fast_gicp

#endif
