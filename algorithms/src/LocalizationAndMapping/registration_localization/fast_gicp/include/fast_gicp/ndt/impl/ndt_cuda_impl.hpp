#ifndef FAST_GICP_NDT_CUDA_IMPL_HPP
#define FAST_GICP_NDT_CUDA_IMPL_HPP

#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <fast_gicp/cuda/ndt_cuda.cuh>

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
NDTCuda<PointSource, PointTarget>::NDTCuda() : LsqRegistration<PointSource, PointTarget>() {
  this->reg_name_ = "NDTCuda";
  ndt_cuda_.reset(new cuda::NDTCudaCore());
}

template <typename PointSource, typename PointTarget>
NDTCuda<PointSource, PointTarget>::~NDTCuda() {}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::setDistanceMode(NDTDistanceMode mode) {
  ndt_cuda_->set_distance_mode(mode);
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::setResolution(double resolution) {
  ndt_cuda_->set_resolution(resolution);
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::setNeighborSearchMethod(NeighborSearchMethod method, double radius) {
  ndt_cuda_->set_neighbor_search_method(method, radius);
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::swapSourceAndTarget() {
  ndt_cuda_->swap_source_and_target();
  input_.swap(target_);
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::clearSource() {
  input_.reset();
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::clearTarget() {
  target_.reset();
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::setInputSource(const PointCloudSourceConstPtr& cloud) {
  if (cloud == input_) {
    return;
  }
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(cloud->size());
  std::transform(cloud->begin(), cloud->end(), points.begin(), [=](const PointSource& pt) { return pt.getVector3fMap(); });
  ndt_cuda_->set_source_cloud(points);
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (cloud == target_) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(cloud->size());
  std::transform(cloud->begin(), cloud->end(), points.begin(), [=](const PointTarget& pt) { return pt.getVector3fMap(); });
  ndt_cuda_->set_target_cloud(points);
}

template <typename PointSource, typename PointTarget>
void NDTCuda<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  ndt_cuda_->create_voxelmaps();
  LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget>
double NDTCuda<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  ndt_cuda_->update_correspondences(trans);
  return ndt_cuda_->compute_error(trans, H, b);
}

template <typename PointSource, typename PointTarget>
double NDTCuda<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  return ndt_cuda_->compute_error(trans, nullptr, nullptr);
}

}  // namespace fast_gicp

#endif