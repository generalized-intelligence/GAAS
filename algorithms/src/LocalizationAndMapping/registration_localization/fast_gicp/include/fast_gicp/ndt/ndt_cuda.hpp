#ifndef FAST_GICP_NDT_CUDA_HPP
#define FAST_GICP_NDT_CUDA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>
#include <fast_gicp/gicp/lsq_registration.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>
#include <fast_gicp/ndt/ndt_settings.hpp>

namespace fast_gicp {

namespace cuda {
class NDTCudaCore;
}

template <typename PointSource, typename PointTarget>
class NDTCuda : public LsqRegistration<PointSource, PointTarget> {
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
  using Ptr = pcl::shared_ptr<NDTCuda<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const NDTCuda<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<NDTCuda<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const NDTCuda<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

public:
  NDTCuda();
  virtual ~NDTCuda() override;

  void setDistanceMode(NDTDistanceMode mode);
  void setResolution(double resolution);
  void setNeighborSearchMethod(NeighborSearchMethod method, double radius = -1.0);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;
  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;
  virtual double compute_error(const Eigen::Isometry3d& trans) override;

protected:
  std::unique_ptr<cuda::NDTCudaCore> ndt_cuda_;
};
}  // namespace fast_gicp

#endif