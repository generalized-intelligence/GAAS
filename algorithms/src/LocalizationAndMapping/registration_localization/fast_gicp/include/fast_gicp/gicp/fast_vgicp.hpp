#ifndef FAST_GICP_FAST_VGICP_HPP
#define FAST_GICP_FAST_VGICP_HPP

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include <fast_gicp/gicp/gicp_settings.hpp>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp_voxel.hpp>

namespace fast_gicp {

/**
 * @brief Fast Voxelized GICP algorithm boosted with OpenMP
 */
template<typename PointSource, typename PointTarget>
class FastVGICP : public FastGICP<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<FastVGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastVGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastVGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastVGICP<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;

  using FastGICP<PointSource, PointTarget>::num_threads_;
  using FastGICP<PointSource, PointTarget>::source_kdtree_;
  using FastGICP<PointSource, PointTarget>::target_kdtree_;
  using FastGICP<PointSource, PointTarget>::source_covs_;
  using FastGICP<PointSource, PointTarget>::target_covs_;

public:
  FastVGICP();
  virtual ~FastVGICP() override;

  void setResolution(double resolution);
  void setVoxelAccumulationMode(VoxelAccumulationMode mode);
  void setNeighborSearchMethod(NeighborSearchMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;
  virtual void update_correspondences(const Eigen::Isometry3d& trans) override;
  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) override;
  virtual double compute_error(const Eigen::Isometry3d& trans) override;

protected:
  double voxel_resolution_;
  NeighborSearchMethod search_method_;
  VoxelAccumulationMode voxel_mode_;

  std::unique_ptr<GaussianVoxelMap<PointTarget>> voxelmap_;

  std::vector<std::pair<int, GaussianVoxel::Ptr>> voxel_correspondences_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> voxel_mahalanobis_;
};
}  // namespace fast_gicp

#endif
