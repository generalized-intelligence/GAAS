#ifndef FAST_GICP_FAST_VGICP_CUDA_HPP
#define FAST_GICP_FAST_VGICP_CUDA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include <fast_gicp/gicp/gicp_settings.hpp>
#include <fast_gicp/gicp/lsq_registration.hpp>

namespace fast_gicp {

namespace cuda {
class FastVGICPCudaCore;
}

enum class NearestNeighborMethod { CPU_PARALLEL_KDTREE, GPU_BRUTEFORCE, GPU_RBF_KERNEL };

/**
 * @brief Fast Voxelized GICP algorithm boosted with CUDA
 */
template<typename PointSource, typename PointTarget>
class FastVGICPCuda : public LsqRegistration<PointSource, PointTarget> {
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
  using Ptr = pcl::shared_ptr<FastVGICPCuda<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastVGICPCuda<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastVGICPCuda<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastVGICPCuda<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;

public:
  FastVGICPCuda();
  virtual ~FastVGICPCuda() override;

  void setCorrespondenceRandomness(int k);
  void setResolution(double resolution);
  void setKernelWidth(double kernel_width, double max_dist = -1.0);
  void setRegularizationMethod(RegularizationMethod method);
  void setNeighborSearchMethod(NeighborSearchMethod method, double radius = -1.0);
  void setNearestNeighborSearchMethod(NearestNeighborMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;
  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) override;
  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  std::vector<int> find_neighbors_parallel_kdtree(int k, typename pcl::PointCloud<PointT>::ConstPtr cloud) const;

private:
  int k_correspondences_;
  double voxel_resolution_;
  RegularizationMethod regularization_method_;
  NearestNeighborMethod neighbor_search_method_;

  std::unique_ptr<cuda::FastVGICPCudaCore> vgicp_cuda_;
};

}  // namespace fast_gicp

#endif
