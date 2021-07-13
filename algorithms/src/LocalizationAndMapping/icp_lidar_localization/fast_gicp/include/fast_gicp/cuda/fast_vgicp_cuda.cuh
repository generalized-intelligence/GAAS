#ifndef FAST_GICP_FAST_VGICP_CUDA_CORE_CUH
#define FAST_GICP_FAST_VGICP_CUDA_CORE_CUH

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fast_gicp/gicp/gicp_settings.hpp>

namespace thrust {

template <typename T1, typename T2>
class pair;

template <typename T>
class device_allocator;

template <typename T, typename Alloc>
class device_vector;
}  // namespace thrust

namespace fast_gicp {
namespace cuda {

class GaussianVoxelMap;

class FastVGICPCudaCore {
public:
  using Points = thrust::device_vector<Eigen::Vector3f, thrust::device_allocator<Eigen::Vector3f>>;
  using Indices = thrust::device_vector<int, thrust::device_allocator<int>>;
  using Matrices = thrust::device_vector<Eigen::Matrix3f, thrust::device_allocator<Eigen::Matrix3f>>;
  using Correspondences = thrust::device_vector<thrust::pair<int, int>, thrust::device_allocator<thrust::pair<int, int>>>;
  using VoxelCoordinates = thrust::device_vector<Eigen::Vector3i, thrust::device_allocator<Eigen::Vector3i>>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FastVGICPCudaCore();
  ~FastVGICPCudaCore();

  void set_resolution(double resolution);
  void set_kernel_params(double kernel_width, double kernel_max_dist);
  void set_neighbor_search_method(fast_gicp::NeighborSearchMethod method, double radius);

  void swap_source_and_target();
  void set_source_cloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cloud);
  void set_target_cloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cloud);

  void set_source_neighbors(int k, const std::vector<int>& neighbors);
  void set_target_neighbors(int k, const std::vector<int>& neighbors);
  void find_source_neighbors(int k);
  void find_target_neighbors(int k);

  void calculate_source_covariances(RegularizationMethod method);
  void calculate_target_covariances(RegularizationMethod method);

  void calculate_source_covariances_rbf(RegularizationMethod method);
  void calculate_target_covariances_rbf(RegularizationMethod method);

  void get_source_covariances(std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs) const;
  void get_target_covariances(std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs) const;

  void get_voxel_num_points(std::vector<int>& num_points) const;
  void get_voxel_means(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& means) const;
  void get_voxel_covs(std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs) const;
  void get_voxel_correspondences(std::vector<std::pair<int, int>>& correspondences) const;

  void create_target_voxelmap();

  void update_correspondences(const Eigen::Isometry3d& trans);

  double compute_error(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) const;

public:
  double resolution;
  double kernel_width;
  double kernel_max_dist;
  std::unique_ptr<VoxelCoordinates> offsets;

  std::unique_ptr<Points> source_points;
  std::unique_ptr<Points> target_points;

  std::unique_ptr<Indices> source_neighbors;
  std::unique_ptr<Indices> target_neighbors;

  std::unique_ptr<Matrices> source_covariances;
  std::unique_ptr<Matrices> target_covariances;

  std::unique_ptr<GaussianVoxelMap> voxelmap;

  Eigen::Isometry3f linearized_x;
  std::unique_ptr<Correspondences> voxel_correspondences;
};

}  // namespace cuda
}  // namespace fast_gicp

#endif