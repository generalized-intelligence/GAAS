#include <fast_gicp/cuda/gaussian_voxelmap.cuh>

#include <fast_gicp/cuda/vector3_hash.cuh>

namespace fast_gicp {
namespace cuda {

// point coord -> voxel coord conversion
struct voxel_coord_kernel {
  voxel_coord_kernel(const thrust::device_ptr<const VoxelMapInfo>& info) : voxelmap_info_ptr(info) {}

  __host__ __device__ Eigen::Vector3i operator()(const Eigen::Vector3f& x) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);
    return calc_voxel_coord(x, info.voxel_resolution);
  }

  const thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
};

// assign voxel indices to buckets
struct voxel_bucket_assignment_kernel {
  voxel_bucket_assignment_kernel(
    const thrust::device_ptr<const VoxelMapInfo>& voxelmap_info,
    const thrust::device_vector<Eigen::Vector3i>& point_coords,
    thrust::device_vector<thrust::pair<int, int>>& index_buckets,
    thrust::device_vector<int>& voxels_failures)
  : voxelmap_info_ptr(voxelmap_info),
    point_coords_ptr(point_coords.data()),
    index_buckets_ptr(index_buckets.data()),
    voxels_failures_ptr(voxels_failures.data()) {}

  __device__ void operator()(int point_index) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);
    const Eigen::Vector3i* coords = thrust::raw_pointer_cast(point_coords_ptr);
    uint64_t hash = vector3i_hash(coords[point_index]);

    for (int i = 0; i < info.max_bucket_scan_count; i++) {
      uint64_t bucket_index = (hash + i) % info.num_buckets;
      thrust::pair<int, int>* index_bucket = thrust::raw_pointer_cast(index_buckets_ptr) + bucket_index;

      int old = atomicCAS(&index_bucket->first, -1, point_index);
      if (old < 0) {
        index_bucket->second = atomicAdd(thrust::raw_pointer_cast(voxels_failures_ptr), 1);
        return;
      }

      if (equal(coords[point_index], coords[old])) {
        return;
      }
    }
    atomicAdd(thrust::raw_pointer_cast(voxels_failures_ptr) + 1, 1);
  }

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const Eigen::Vector3i> point_coords_ptr;
  thrust::device_ptr<thrust::pair<int, int>> index_buckets_ptr;
  thrust::device_ptr<int> voxels_failures_ptr;
};

// pair<point index, bucket index>  to pair<voxel coord, bucket index>
struct voxel_coord_select_kernel {
  voxel_coord_select_kernel(const thrust::device_vector<Eigen::Vector3i>& point_coords) : point_coords_ptr(point_coords.data()) {}

  __device__ thrust::pair<Eigen::Vector3i, int> operator()(const thrust::pair<int, int>& index_bucket) const {
    if (index_bucket.first < 0) {
      return thrust::make_pair(Eigen::Vector3i(0, 0, 0), -1);
    }

    return thrust::make_pair(thrust::raw_pointer_cast(point_coords_ptr)[index_bucket.first], index_bucket.second);
  }

  thrust::device_ptr<const Eigen::Vector3i> point_coords_ptr;
};

// accumulate points and covs
struct accumulate_points_kernel {
  accumulate_points_kernel(
    const thrust::device_ptr<VoxelMapInfo>& voxelmap_info_ptr,
    const thrust::device_vector<thrust::pair<Eigen::Vector3i, int>>& buckets,
    thrust::device_vector<int>& num_points,
    thrust::device_vector<Eigen::Vector3f>& voxel_means,
    thrust::device_vector<Eigen::Matrix3f>& voxel_covs)
  : voxelmap_info_ptr(voxelmap_info_ptr),
    buckets_ptr(buckets.data()),
    num_points_ptr(num_points.data()),
    voxel_means_ptr(voxel_means.data()),
    voxel_covs_ptr(voxel_covs.data()) {}

  __device__ void operator()(const thrust::tuple<Eigen::Vector3f, Eigen::Matrix3f>& input) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);

    const auto& mean = thrust::get<0>(input);
    const auto& cov = thrust::get<1>(input);

    const Eigen::Vector3i coord = calc_voxel_coord(mean, info.voxel_resolution);
    uint64_t hash = vector3i_hash(coord);

    for (int i = 0; i < info.max_bucket_scan_count; i++) {
      uint64_t bucket_index = (hash + i) % info.num_buckets;
      const thrust::pair<Eigen::Vector3i, int>& bucket = thrust::raw_pointer_cast(buckets_ptr)[bucket_index];

      if (equal(bucket.first, coord)) {
        int& num_points = thrust::raw_pointer_cast(num_points_ptr)[bucket.second];
        Eigen::Vector3f& voxel_mean = thrust::raw_pointer_cast(voxel_means_ptr)[bucket.second];
        Eigen::Matrix3f& voxel_cov = thrust::raw_pointer_cast(voxel_covs_ptr)[bucket.second];

        atomicAdd(&num_points, 1);
        for (int j = 0; j < 3; j++) {
          atomicAdd(voxel_mean.data() + j, mean[j]);
        }
        for (int j = 0; j < 9; j++) {
          atomicAdd(voxel_cov.data() + j, cov.data()[j]);
        }
      }
    }
  }

  __device__ void operator()(const Eigen::Vector3f& mean) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);

    const Eigen::Vector3i coord = calc_voxel_coord(mean, info.voxel_resolution);
    uint64_t hash = vector3i_hash(coord);

    for (int i = 0; i < info.max_bucket_scan_count; i++) {
      uint64_t bucket_index = (hash + i) % info.num_buckets;
      const thrust::pair<Eigen::Vector3i, int>& bucket = thrust::raw_pointer_cast(buckets_ptr)[bucket_index];

      if (equal(bucket.first, coord)) {
        int& num_points = thrust::raw_pointer_cast(num_points_ptr)[bucket.second];
        Eigen::Vector3f& voxel_mean = thrust::raw_pointer_cast(voxel_means_ptr)[bucket.second];
        Eigen::Matrix3f& voxel_cov = thrust::raw_pointer_cast(voxel_covs_ptr)[bucket.second];

        Eigen::Matrix3f cov = mean * mean.transpose();

        atomicAdd(&num_points, 1);
        for (int j = 0; j < 3; j++) {
          atomicAdd(voxel_mean.data() + j, mean[j]);
        }
        for (int j = 0; j < 9; j++) {
          atomicAdd(voxel_cov.data() + j, cov.data()[j]);
        }
      }
    }
  }

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const thrust::pair<Eigen::Vector3i, int>> buckets_ptr;

  thrust::device_ptr<int> num_points_ptr;
  thrust::device_ptr<Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<Eigen::Matrix3f> voxel_covs_ptr;
};

struct finalize_voxels_kernel {
  finalize_voxels_kernel(thrust::device_vector<int>& num_points, thrust::device_vector<Eigen::Vector3f>& voxel_means, thrust::device_vector<Eigen::Matrix3f>& voxel_covs)
  : num_points_ptr(num_points.data()),
    voxel_means_ptr(voxel_means.data()),
    voxel_covs_ptr(voxel_covs.data()) {}

  __host__ __device__ void operator()(int i) const {
    int num_points = thrust::raw_pointer_cast(num_points_ptr)[i];
    auto& voxel_mean = thrust::raw_pointer_cast(voxel_means_ptr)[i];
    auto& voxel_covs = thrust::raw_pointer_cast(voxel_covs_ptr)[i];

    voxel_mean /= num_points;
    voxel_covs /= num_points;
  }

  thrust::device_ptr<int> num_points_ptr;
  thrust::device_ptr<Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<Eigen::Matrix3f> voxel_covs_ptr;
};

struct ndt_finalize_voxels_kernel {
  ndt_finalize_voxels_kernel(thrust::device_vector<int>& num_points, thrust::device_vector<Eigen::Vector3f>& voxel_means, thrust::device_vector<Eigen::Matrix3f>& voxel_covs)
  : num_points_ptr(num_points.data()),
    voxel_means_ptr(voxel_means.data()),
    voxel_covs_ptr(voxel_covs.data()) {}

  __host__ __device__ void operator()(int i) const {
    int num_points = thrust::raw_pointer_cast(num_points_ptr)[i];
    auto& voxel_mean = thrust::raw_pointer_cast(voxel_means_ptr)[i];
    auto& voxel_covs = thrust::raw_pointer_cast(voxel_covs_ptr)[i];

    Eigen::Vector3f sum_pts = voxel_mean;

    voxel_mean /= num_points;
    voxel_covs = (voxel_covs - voxel_mean * sum_pts.transpose()) / num_points;
  }

  thrust::device_ptr<int> num_points_ptr;
  thrust::device_ptr<Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<Eigen::Matrix3f> voxel_covs_ptr;
};

GaussianVoxelMap::GaussianVoxelMap(float resolution, int init_num_buckets, int max_bucket_scan_count) : init_num_buckets(init_num_buckets) {
  voxelmap_info.num_voxels = 0;
  voxelmap_info.num_buckets = init_num_buckets;
  voxelmap_info.max_bucket_scan_count = max_bucket_scan_count;
  voxelmap_info.voxel_resolution = resolution;
  voxelmap_info_ptr.resize(1);
  voxelmap_info_ptr[0] = voxelmap_info;
}

void GaussianVoxelMap::create_voxelmap(const thrust::device_vector<Eigen::Vector3f>& points) {
  cudaStream_t stream;
  cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);

  create_bucket_table(stream, points);

  num_points.resize(voxelmap_info.num_voxels);
  voxel_means.resize(voxelmap_info.num_voxels);
  voxel_covs.resize(voxelmap_info.num_voxels);
  num_points.resize(voxelmap_info.num_voxels);
  voxel_means.resize(voxelmap_info.num_voxels);
  voxel_covs.resize(voxelmap_info.num_voxels);
  thrust::fill(thrust::cuda::par.on(stream), num_points.begin(), num_points.end(), 0);
  thrust::fill(thrust::cuda::par.on(stream), voxel_means.begin(), voxel_means.end(), Eigen::Vector3f::Zero().eval());
  thrust::fill(thrust::cuda::par.on(stream), voxel_covs.begin(), voxel_covs.end(), Eigen::Matrix3f::Zero().eval());

  thrust::for_each(thrust::cuda::par.on(stream), points.begin(), points.end(), accumulate_points_kernel(voxelmap_info_ptr.data(), buckets, num_points, voxel_means, voxel_covs));

  thrust::for_each(thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(voxelmap_info.num_voxels), ndt_finalize_voxels_kernel(num_points, voxel_means, voxel_covs));

  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);
}

void GaussianVoxelMap::create_voxelmap(const thrust::device_vector<Eigen::Vector3f>& points, const thrust::device_vector<Eigen::Matrix3f>& covariances) {
  cudaStream_t stream;
  cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);

  create_bucket_table(stream, points);

  num_points.resize(voxelmap_info.num_voxels);
  voxel_means.resize(voxelmap_info.num_voxels);
  voxel_covs.resize(voxelmap_info.num_voxels);
  thrust::fill(thrust::cuda::par.on(stream), num_points.begin(), num_points.end(), 0);
  thrust::fill(thrust::cuda::par.on(stream), voxel_means.begin(), voxel_means.end(), Eigen::Vector3f::Zero().eval());
  thrust::fill(thrust::cuda::par.on(stream), voxel_covs.begin(), voxel_covs.end(), Eigen::Matrix3f::Zero().eval());

  thrust::for_each(
    thrust::cuda::par.on(stream),
    thrust::make_zip_iterator(thrust::make_tuple(points.begin(), covariances.begin())),
    thrust::make_zip_iterator(thrust::make_tuple(points.end(), covariances.end())),
    accumulate_points_kernel(voxelmap_info_ptr.data(), buckets, num_points, voxel_means, voxel_covs));

  thrust::for_each(thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(voxelmap_info.num_voxels), finalize_voxels_kernel(num_points, voxel_means, voxel_covs));

  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);
}

void GaussianVoxelMap::create_bucket_table(cudaStream_t stream, const thrust::device_vector<Eigen::Vector3f>& points) {
  thrust::device_vector<Eigen::Vector3i> coords(points.size());
  thrust::transform(thrust::cuda::par.on(stream), points.begin(), points.end(), coords.begin(), voxel_coord_kernel(voxelmap_info_ptr.data()));

  thrust::device_vector<thrust::pair<int, int>> index_buckets;
  thrust::device_vector<int> voxels_failures(2, 0);

  for (int num_buckets = init_num_buckets; init_num_buckets * 4; num_buckets *= 2) {
    voxelmap_info.num_buckets = num_buckets;
    voxelmap_info_ptr[0] = voxelmap_info;

    index_buckets.resize(num_buckets);
    thrust::fill(thrust::cuda::par.on(stream), index_buckets.begin(), index_buckets.end(), thrust::make_pair(-1, -1));
    thrust::fill(thrust::cuda::par.on(stream), voxels_failures.begin(), voxels_failures.end(), 0);

    thrust::for_each(
      thrust::cuda::par.on(stream),
      thrust::counting_iterator<int>(0),
      thrust::counting_iterator<int>(points.size()),
      voxel_bucket_assignment_kernel(voxelmap_info_ptr.data(), coords, index_buckets, voxels_failures));

    thrust::host_vector<int> h_voxels_failures = voxels_failures;
    if (static_cast<double>(h_voxels_failures[1]) / points.size() < 0.01) {
      voxelmap_info.num_voxels = h_voxels_failures[0];
      voxelmap_info_ptr[0] = voxelmap_info;
      break;
    }
  }

  buckets.resize(index_buckets.size());
  thrust::transform(thrust::cuda::par.on(stream), index_buckets.begin(), index_buckets.end(), buckets.begin(), voxel_coord_select_kernel(coords));
}

}  // namespace cuda
}  // namespace fast_gicp
