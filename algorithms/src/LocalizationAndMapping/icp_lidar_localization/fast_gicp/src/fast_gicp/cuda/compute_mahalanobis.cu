#include <fast_gicp/cuda/compute_mahalanobis.cuh>

#include <fstream>

namespace fast_gicp {
  namespace cuda {

namespace {

struct compute_mahalanobis_kernel {
  compute_mahalanobis_kernel(const GaussianVoxelMap& voxelmap, const Eigen::Isometry3f& x)
  : R(x.linear()),
    t(x.translation()),
    voxel_num_points_ptr(voxelmap.num_points.data()),
    voxel_means_ptr(voxelmap.voxel_means.data()),
    voxel_covs_ptr(voxelmap.voxel_covs.data())
  {}

  __host__ __device__
  Eigen::Matrix3f operator() (const thrust::tuple<Eigen::Vector3f, Eigen::Matrix3f, int>& input) const {
    const Eigen::Vector3f& mean_A = thrust::get<0>(input);
    const Eigen::Matrix3f& cov_A = thrust::get<1>(input);
    int voxel_index = thrust::get<2>(input);

    if(voxel_index < 0) {
      return Eigen::Matrix3f::Identity();
    }

    const Eigen::Matrix3f& cov_B = thrust::raw_pointer_cast(voxel_covs_ptr)[voxel_index];
    Eigen::Matrix3f RCR = cov_B + R * cov_A* R.transpose();
    return RCR.inverse();
  }

  const Eigen::Matrix3f R;
  const Eigen::Vector3f t;

  thrust::device_ptr<const int> voxel_num_points_ptr;
  thrust::device_ptr<const Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<const Eigen::Matrix3f> voxel_covs_ptr;
};
}


void compute_mahalanobis(
  const thrust::device_vector<Eigen::Vector3f>& src_points,
  const thrust::device_vector<Eigen::Matrix3f>& src_covs,
  const GaussianVoxelMap& voxelmap,
  const thrust::device_vector<int>& voxel_correspondences,
  const Eigen::Isometry3f& linearized_x,
  thrust::device_vector<Eigen::Matrix3f>& mahalanobis
) {
  mahalanobis.resize(src_points.size());
  thrust::transform(
    thrust::make_zip_iterator(thrust::make_tuple(src_points.begin(), src_covs.begin(), voxel_correspondences.begin())),
    thrust::make_zip_iterator(thrust::make_tuple(src_points.end(), src_covs.end(), voxel_correspondences.end())),
    mahalanobis.begin(),
    compute_mahalanobis_kernel(voxelmap, linearized_x)
  );

  thrust::host_vector<int> corrs = voxel_correspondences;
  thrust::host_vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> h_maha = mahalanobis;
  std::ofstream ofs("/tmp/vgicp_cuda_mahalanobis.txt");
  for(int i=0; i<corrs.size(); i++) {
    if(corrs[i] < 0) {
      continue;
    }

    ofs << i << std::endl << h_maha[i] << std::endl;
  }

  // exit(0);
}

  }
}