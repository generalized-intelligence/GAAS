#ifndef FAST_GICP_FAST_GICP_ST_HPP
#define FAST_GICP_FAST_GICP_ST_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>

namespace fast_gicp {

/**
 * @brief Fast GICP algorithm optimized for single threading
 */
template<typename PointSource, typename PointTarget>
class FastGICPSingleThread : public FastGICP<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<FastGICPSingleThread<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastGICPSingleThread<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastGICPSingleThread<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastGICPSingleThread<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;

  using FastGICP<PointSource, PointTarget>::target_kdtree_;
  using FastGICP<PointSource, PointTarget>::correspondences_;
  using FastGICP<PointSource, PointTarget>::sq_distances_;
  using FastGICP<PointSource, PointTarget>::source_covs_;
  using FastGICP<PointSource, PointTarget>::target_covs_;
  using FastGICP<PointSource, PointTarget>::mahalanobis_;

public:
  FastGICPSingleThread();
  virtual ~FastGICPSingleThread() override;

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

private:
  virtual void update_correspondences(const Eigen::Isometry3d& trans) override;

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

private:
  std::vector<float> second_sq_distances_;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> anchors_;
};
}  // namespace fast_gicp

#endif
