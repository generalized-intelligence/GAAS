#ifndef FAST_GICP_NDT_CUDA_CUH
#define FAST_GICP_NDT_CUDA_CUH

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fast_gicp/ndt/ndt_settings.hpp>
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

class NDTCudaCore {
public:
  using Points = thrust::device_vector<Eigen::Vector3f, thrust::device_allocator<Eigen::Vector3f>>;
  using Indices = thrust::device_vector<int, thrust::device_allocator<int>>;
  using Matrices = thrust::device_vector<Eigen::Matrix3f, thrust::device_allocator<Eigen::Matrix3f>>;
  using Correspondences = thrust::device_vector<thrust::pair<int, int>, thrust::device_allocator<thrust::pair<int, int>>>;
  using VoxelCoordinates = thrust::device_vector<Eigen::Vector3i, thrust::device_allocator<Eigen::Vector3i>>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NDTCudaCore();
  ~NDTCudaCore();

  void set_distance_mode(fast_gicp::NDTDistanceMode mode);
  void set_resolution(double resolution);
  void set_neighbor_search_method(fast_gicp::NeighborSearchMethod method, double radius);

  void swap_source_and_target();
  void set_source_cloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cloud);
  void set_target_cloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cloud);

  void create_voxelmaps();
  void create_target_voxelmap();
  void create_source_voxelmap();

  void update_correspondences(const Eigen::Isometry3d& trans);
  double compute_error(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) const;

public:
  fast_gicp::NDTDistanceMode distance_mode;
  double resolution;
  std::unique_ptr<VoxelCoordinates> offsets;

  std::unique_ptr<Points> source_points;
  std::unique_ptr<Points> target_points;

  std::unique_ptr<GaussianVoxelMap> source_voxelmap;
  std::unique_ptr<GaussianVoxelMap> target_voxelmap;

  Eigen::Isometry3f linearized_x;
  std::unique_ptr<Correspondences> correspondences;
};

}  // namespace cuda
}  // namespace fast_gicp

#endif