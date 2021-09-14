#include <Eigen/Core>
#include <Eigen/Geometry>

#include <thrust/remove.h>
#include <thrust/device_vector.h>
#include <thrust/async/transform.h>
#include <fast_gicp/cuda/vector3_hash.cuh>
#include <fast_gicp/cuda/gaussian_voxelmap.cuh>
#include <fast_gicp/cuda/find_voxel_correspondences.cuh>

namespace fast_gicp {
namespace cuda {

namespace {

struct find_voxel_correspondences_kernel {
  find_voxel_correspondences_kernel(
    const GaussianVoxelMap& voxelmap,
    const thrust::device_vector<Eigen::Vector3f>& src_points,
    const thrust::device_ptr<const Eigen::Isometry3f>& trans_ptr,
    thrust::device_ptr<const Eigen::Vector3i> offset_ptr)
  : trans_ptr(trans_ptr),
    offset_ptr(offset_ptr),
    src_points_ptr(src_points.data()),
    voxelmap_info_ptr(voxelmap.voxelmap_info_ptr.data()),
    buckets_ptr(voxelmap.buckets.data()),
    voxel_num_points_ptr(voxelmap.num_points.data()),
    voxel_means_ptr(voxelmap.voxel_means.data()),
    voxel_covs_ptr(voxelmap.voxel_covs.data()) {}

  // lookup voxel
  __host__ __device__ int lookup_voxel(const Eigen::Vector3f& x) const {
    const VoxelMapInfo& voxelmap_info = *thrust::raw_pointer_cast(voxelmap_info_ptr);
    const auto& offset = *thrust::raw_pointer_cast(offset_ptr);

    Eigen::Vector3i coord = calc_voxel_coord(x, voxelmap_info.voxel_resolution) + offset;
    uint64_t hash = vector3i_hash(coord);

    for(int i = 0; i < voxelmap_info.max_bucket_scan_count; i++) {
      uint64_t bucket_index = (hash + i) % voxelmap_info.num_buckets;
      const thrust::pair<Eigen::Vector3i, int>& bucket = thrust::raw_pointer_cast(buckets_ptr)[bucket_index];

      if(bucket.second < 0) {
        return -1;
      }

      if(bucket.first == coord) {
        return bucket.second;
      }
    }

    return -1;
  }

  __host__ __device__ thrust::pair<int, int> operator()(int src_index) const {
    const auto& trans = *thrust::raw_pointer_cast(trans_ptr);

    const auto& pt = thrust::raw_pointer_cast(src_points_ptr)[src_index];
    return thrust::make_pair(src_index, lookup_voxel(trans.linear() * pt + trans.translation()));
  }

  const thrust::device_ptr<const Eigen::Isometry3f> trans_ptr;
  const thrust::device_ptr<const Eigen::Vector3i> offset_ptr;

  const thrust::device_ptr<const Eigen::Vector3f> src_points_ptr;

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const thrust::pair<Eigen::Vector3i, int>> buckets_ptr;

  thrust::device_ptr<const int> voxel_num_points_ptr;
  thrust::device_ptr<const Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<const Eigen::Matrix3f> voxel_covs_ptr;
};

struct invalid_correspondence_kernel {
  __host__ __device__ bool operator() (const thrust::pair<int, int>& corr) const {
    return corr.first < 0 || corr.second < 0;
  }
};

}  // namespace

void find_voxel_correspondences(
  const thrust::device_vector<Eigen::Vector3f>& src_points,
  const GaussianVoxelMap& voxelmap,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr,
  const thrust::device_vector<Eigen::Vector3i>& offsets,
  thrust::device_vector<thrust::pair<int, int>>& correspondences) {
  std::vector<thrust::system::cuda::unique_eager_event> events(offsets.size());

  // find correspondences
  correspondences.resize(src_points.size() * offsets.size());
  for(int i=0; i<offsets.size(); i++) {
    auto event = thrust::async::transform(
      thrust::counting_iterator<int>(0),
      thrust::counting_iterator<int>(src_points.size()),
      correspondences.begin() + src_points.size() * i,
      find_voxel_correspondences_kernel(voxelmap, src_points, x_ptr, offsets.data() + i));

    events[i] = std::move(event);
  }

  // synchronize
  for(auto& event: events) {
    event.wait();
  }

  // remove invlid correspondences
  auto remove_loc = thrust::remove_if(correspondences.begin(), correspondences.end(), invalid_correspondence_kernel());
  correspondences.erase(remove_loc, correspondences.end());
}

}  // namespace cuda
}  // namespace fast_gicp
