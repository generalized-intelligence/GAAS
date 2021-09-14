#ifndef FAST_GICP_CUDA_GAUSSIAN_VOXELMAP_CUH
#define FAST_GICP_CUDA_GAUSSIAN_VOXELMAP_CUH

#include <Eigen/Core>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace fast_gicp {
namespace cuda {

struct VoxelMapInfo {
  int num_voxels;
  int num_buckets;
  int max_bucket_scan_count;
  float voxel_resolution;
};

class GaussianVoxelMap {
public:
  GaussianVoxelMap(float resolution, int init_num_buckets = 8192, int max_bucket_scan_count = 10);

  void create_voxelmap(const thrust::device_vector<Eigen::Vector3f>& points);
  void create_voxelmap(const thrust::device_vector<Eigen::Vector3f>& points, const thrust::device_vector<Eigen::Matrix3f>& covariances);

private:
  void create_bucket_table(cudaStream_t stream, const thrust::device_vector<Eigen::Vector3f>& points);

public:
  const int init_num_buckets;
  VoxelMapInfo voxelmap_info;
  thrust::device_vector<VoxelMapInfo> voxelmap_info_ptr;

  thrust::device_vector<thrust::pair<Eigen::Vector3i, int>> buckets;

  // voxel data
  thrust::device_vector<int> num_points;
  thrust::device_vector<Eigen::Vector3f> voxel_means;
  thrust::device_vector<Eigen::Matrix3f> voxel_covs;
};

}  // namespace cuda
}  // namespace fast_gicp

#endif