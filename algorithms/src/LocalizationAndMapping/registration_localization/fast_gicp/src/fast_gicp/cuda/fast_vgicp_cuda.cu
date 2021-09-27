#include <fast_gicp/cuda/fast_vgicp_cuda.cuh>

#include <thrust/device_new.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <fast_gicp/cuda/brute_force_knn.cuh>
#include <fast_gicp/cuda/covariance_estimation.cuh>
#include <fast_gicp/cuda/covariance_regularization.cuh>
#include <fast_gicp/cuda/gaussian_voxelmap.cuh>
#include <fast_gicp/cuda/compute_mahalanobis.cuh>
#include <fast_gicp/cuda/compute_derivatives.cuh>
#include <fast_gicp/cuda/find_voxel_correspondences.cuh>

namespace fast_gicp {
namespace cuda {

FastVGICPCudaCore::FastVGICPCudaCore() {
  // warming up GPU
  cudaDeviceSynchronize();

  resolution = 1.0;
  linearized_x.setIdentity();

  kernel_width = 0.25;
  kernel_max_dist = 3.0;

  offsets.reset(new thrust::device_vector<Eigen::Vector3i>(1));
  (*offsets)[0] = Eigen::Vector3i::Zero().eval();
}
FastVGICPCudaCore ::~FastVGICPCudaCore() {}

void FastVGICPCudaCore::set_resolution(double resolution) {
  this->resolution = resolution;
}

void FastVGICPCudaCore::set_kernel_params(double kernel_width, double kernel_max_dist) {
  this->kernel_width = kernel_width;
  this->kernel_max_dist = kernel_max_dist;
}

void FastVGICPCudaCore::set_neighbor_search_method(fast_gicp::NeighborSearchMethod method, double radius) {
  thrust::host_vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> h_offsets;

  switch (method) {
    default:
      std::cerr << "here must not be reached" << std::endl;
      abort();

    case fast_gicp::NeighborSearchMethod::DIRECT1:
      h_offsets.resize(1);
      h_offsets[0] = Eigen::Vector3i::Zero();
      break;

    case fast_gicp::NeighborSearchMethod::DIRECT7:
      h_offsets.resize(7);
      h_offsets[0] = Eigen::Vector3i(0, 0, 0);
      h_offsets[1] = Eigen::Vector3i(1, 0, 0);
      h_offsets[2] = Eigen::Vector3i(-1, 0, 0);
      h_offsets[3] = Eigen::Vector3i(0, 1, 0);
      h_offsets[4] = Eigen::Vector3i(0, -1, 0);
      h_offsets[5] = Eigen::Vector3i(0, 0, 1);
      h_offsets[6] = Eigen::Vector3i(0, 0, -1);
      break;

    case fast_gicp::NeighborSearchMethod::DIRECT27:
      h_offsets.reserve(27);
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          for (int k = 0; k < 3; k++) {
            h_offsets.push_back(Eigen::Vector3i(i - 1, j - 1, k - 1));
          }
        }
      }
      break;

    case fast_gicp::NeighborSearchMethod::DIRECT_RADIUS:
      h_offsets.reserve(50);
      int range = std::ceil(radius);
      for (int i = -range; i <= range; i++) {
        for (int j = -range; j <= range; j++) {
          for (int k = -range; k <= range; k++) {
            Eigen::Vector3i offset(i, j, k);
            if(offset.cast<double>().norm() <= radius + 1e-3) {
              h_offsets.push_back(offset);
            }
          }
        }
      }

      break;
  }

  *offsets = h_offsets;
}

void FastVGICPCudaCore::swap_source_and_target() {
  source_points.swap(target_points);
  source_neighbors.swap(target_neighbors);
  source_covariances.swap(target_covariances);

  if(!target_points || !target_covariances) {
    return;
  }

  create_target_voxelmap();
}

void FastVGICPCudaCore::set_source_cloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cloud) {
  thrust::host_vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(cloud.begin(), cloud.end());
  if(!source_points) {
    source_points.reset(new Points());
  }

  *source_points = points;
}

void FastVGICPCudaCore::set_target_cloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cloud) {
  thrust::host_vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(cloud.begin(), cloud.end());
  if(!target_points) {
    target_points.reset(new Points());
  }

  *target_points = points;
}

void FastVGICPCudaCore::set_source_neighbors(int k, const std::vector<int>& neighbors) {
  assert(k * source_points->size() == neighbors.size());
  thrust::host_vector<int> k_neighbors(neighbors.begin(), neighbors.end());

  if(!source_neighbors) {
    source_neighbors.reset(new thrust::device_vector<int>());
  }

  *source_neighbors = k_neighbors;
}

void FastVGICPCudaCore::set_target_neighbors(int k, const std::vector<int>& neighbors) {
  assert(k * target_points->size() == neighbors.size());
  thrust::host_vector<int> k_neighbors(neighbors.begin(), neighbors.end());

  if(!target_neighbors) {
    target_neighbors.reset(new thrust::device_vector<int>());
  }

  *target_neighbors = k_neighbors;
}

struct untie_pair_second {
  __device__ int operator()(thrust::pair<float, int>& p) const {
    return p.second;
  }
};

void FastVGICPCudaCore::find_source_neighbors(int k) {
  assert(source_points);

  thrust::device_vector<thrust::pair<float, int>> k_neighbors;
  brute_force_knn_search(*source_points, *source_points, k, k_neighbors);

  if(!source_neighbors) {
    source_neighbors.reset(new thrust::device_vector<int>(k_neighbors.size()));
  } else {
    source_neighbors->resize(k_neighbors.size());
  }
  thrust::transform(k_neighbors.begin(), k_neighbors.end(), source_neighbors->begin(), untie_pair_second());
}

void FastVGICPCudaCore::find_target_neighbors(int k) {
  assert(target_points);

  thrust::device_vector<thrust::pair<float, int>> k_neighbors;
  brute_force_knn_search(*target_points, *target_points, k, k_neighbors);

  if(!target_neighbors) {
    target_neighbors.reset(new thrust::device_vector<int>(k_neighbors.size()));
  } else {
    target_neighbors->resize(k_neighbors.size());
  }
  thrust::transform(k_neighbors.begin(), k_neighbors.end(), target_neighbors->begin(), untie_pair_second());
}

void FastVGICPCudaCore::calculate_source_covariances(RegularizationMethod method) {
  assert(source_points && source_neighbors);
  int k = source_neighbors->size() / source_points->size();

  if(!source_covariances) {
    source_covariances.reset(new thrust::device_vector<Eigen::Matrix3f>(source_points->size()));
  }
  covariance_estimation(*source_points, k, *source_neighbors, *source_covariances);
  covariance_regularization(*source_points, *source_covariances, method);
}

void FastVGICPCudaCore::calculate_target_covariances(RegularizationMethod method) {
  assert(target_points && target_neighbors);
  int k = target_neighbors->size() / target_points->size();

  if(!target_covariances) {
    target_covariances.reset(new thrust::device_vector<Eigen::Matrix3f>(target_points->size()));
  }
  covariance_estimation(*target_points, k, *target_neighbors, *target_covariances);
  covariance_regularization(*target_points, *target_covariances, method);
}

void FastVGICPCudaCore::calculate_source_covariances_rbf(RegularizationMethod method) {
  if(!source_covariances) {
    source_covariances.reset(new thrust::device_vector<Eigen::Matrix3f>(source_points->size()));
  }
  covariance_estimation_rbf(*source_points, kernel_width, kernel_max_dist, *source_covariances);
  covariance_regularization(*source_points, *source_covariances, method);
}

void FastVGICPCudaCore::calculate_target_covariances_rbf(RegularizationMethod method) {
  if(!target_covariances) {
    target_covariances.reset(new thrust::device_vector<Eigen::Matrix3f>(target_points->size()));
  }
  covariance_estimation_rbf(*target_points, kernel_width, kernel_max_dist, *target_covariances);
  covariance_regularization(*target_points, *target_covariances, method);
}

void FastVGICPCudaCore::get_voxel_correspondences(std::vector<std::pair<int, int>>& correspondences) const {
  thrust::host_vector<thrust::pair<int, int>> corrs = *voxel_correspondences;
  correspondences.resize(corrs.size());
  std::transform(corrs.begin(), corrs.end(), correspondences.begin(), [](const auto& x) { return std::make_pair(x.first, x.second); });
}

void FastVGICPCudaCore::get_voxel_num_points(std::vector<int>& num_points) const {
  thrust::host_vector<int> voxel_num_points = voxelmap->num_points;
  num_points.resize(voxel_num_points.size());
  std::copy(voxel_num_points.begin(), voxel_num_points.end(), num_points.begin());
}

void FastVGICPCudaCore::get_voxel_means(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& means) const {
  thrust::host_vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> voxel_means = voxelmap->voxel_means;
  means.resize(voxel_means.size());
  std::copy(voxel_means.begin(), voxel_means.end(), means.begin());
}

void FastVGICPCudaCore::get_voxel_covs(std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs) const {
  thrust::host_vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> voxel_covs = voxelmap->voxel_covs;
  covs.resize(voxel_covs.size());
  std::copy(voxel_covs.begin(), voxel_covs.end(), covs.begin());
}

void FastVGICPCudaCore::get_source_covariances(std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs) const {
  thrust::host_vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> c = *source_covariances;
  covs.resize(c.size());
  std::copy(c.begin(), c.end(), covs.begin());
}

void FastVGICPCudaCore::get_target_covariances(std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs) const {
  thrust::host_vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> c = *target_covariances;
  covs.resize(c.size());
  std::copy(c.begin(), c.end(), covs.begin());
}

void FastVGICPCudaCore::create_target_voxelmap() {
  assert(target_points && target_covariances);
  if(!voxelmap) {
    voxelmap.reset(new GaussianVoxelMap(resolution));
  }
  voxelmap->create_voxelmap(*target_points, *target_covariances);
}

void FastVGICPCudaCore::update_correspondences(const Eigen::Isometry3d& trans) {
  thrust::device_vector<Eigen::Isometry3f> trans_ptr(1);
  trans_ptr[0] = trans.cast<float>();

  if(voxel_correspondences == nullptr) {
    voxel_correspondences.reset(new Correspondences());
  }
  linearized_x = trans.cast<float>();
  find_voxel_correspondences(*source_points, *voxelmap, trans_ptr.data(), *offsets, *voxel_correspondences);
}

double FastVGICPCudaCore::compute_error(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) const {
  thrust::host_vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> trans_(2);
  trans_[0] = linearized_x;
  trans_[1] = trans.cast<float>();

  thrust::device_vector<Eigen::Isometry3f> trans_ptr = trans_;

  return compute_derivatives(*source_points, *source_covariances, *voxelmap, *voxel_correspondences, trans_ptr.data(), trans_ptr.data() + 1, H, b);
}

}  // namespace cuda
}  // namespace fast_gicp
