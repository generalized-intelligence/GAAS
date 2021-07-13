#ifndef FAST_GICP_FAST_VGICP_VOXEL_HPP
#define FAST_GICP_FAST_VGICP_VOXEL_HPP

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>

namespace fast_gicp {

static std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> neighbor_offsets(NeighborSearchMethod search_method) {
  switch(search_method) {
      // clang-format off
    default:
      std::cerr << "unsupported neighbor search method" << std::endl;
      abort();
    case NeighborSearchMethod::DIRECT1:
      return std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>{
        Eigen::Vector3i(0, 0, 0)
      };
    case NeighborSearchMethod::DIRECT7:
      return std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>{
        Eigen::Vector3i(0, 0, 0),
        Eigen::Vector3i(1, 0, 0),
        Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0),
        Eigen::Vector3i(0, -1, 0),
        Eigen::Vector3i(0, 0, 1),
        Eigen::Vector3i(0, 0, -1)
      };
    case NeighborSearchMethod::DIRECT27:
      break;
      // clang-format on
  }

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> offsets27;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      for(int k = 0; k < 3; k++) {
        offsets27.push_back(Eigen::Vector3i(i - 1, j - 1, k - 1));
      }
    }
  }
  return offsets27;
}

class Vector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, x[0]);
    boost::hash_combine(seed, x[1]);
    boost::hash_combine(seed, x[2]);
    return seed;
  }
};

struct GaussianVoxel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<GaussianVoxel>;

  GaussianVoxel() {
    num_points = 0;
    mean.setZero();
    cov.setZero();
  }
  virtual ~GaussianVoxel() {}

  virtual void append(const Eigen::Vector4d& mean_, const Eigen::Matrix4d& cov_) = 0;

  virtual void finalize() = 0;

public:
  int num_points;
  Eigen::Vector4d mean;
  Eigen::Matrix4d cov;
};

struct MultiplicativeGaussianVoxel : GaussianVoxel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MultiplicativeGaussianVoxel() : GaussianVoxel() {}
  virtual ~MultiplicativeGaussianVoxel() {}

  virtual void append(const Eigen::Vector4d& mean_, const Eigen::Matrix4d& cov_) override {
    num_points++;
    Eigen::Matrix4d cov_inv = cov_;
    cov_inv(3, 3) = 1;
    cov_inv = cov_inv.inverse().eval();

    cov += cov_inv;
    mean += cov_inv * mean_;
  }

  virtual void finalize() override {
    cov(3, 3) = 1;
    mean[3] = 1;

    cov = cov.inverse().eval();
    mean = (cov * mean).eval();
  }
};

struct AdditiveGaussianVoxel : GaussianVoxel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AdditiveGaussianVoxel() : GaussianVoxel() {}
  virtual ~AdditiveGaussianVoxel() {}

  virtual void append(const Eigen::Vector4d& mean_, const Eigen::Matrix4d& cov_) override {
    num_points++;
    mean += mean_;
    cov += cov_;
  }

  virtual void finalize() override {
    mean /= num_points;
    cov /= num_points;
  }
};

template<typename PointT>
class GaussianVoxelMap {
public:
  GaussianVoxelMap(double resolution, VoxelAccumulationMode mode) : voxel_resolution_(resolution), voxel_mode_(mode) {}

  void create_voxelmap(const pcl::PointCloud<PointT>& cloud, const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
    voxels_.clear();
    for(int i = 0; i < cloud.size(); i++) {
      Eigen::Vector3i coord = voxel_coord(cloud.at(i).getVector4fMap().template cast<double>());

      auto found = voxels_.find(coord);
      if(found == voxels_.end()) {
        GaussianVoxel::Ptr voxel;
        switch(voxel_mode_) {
          case VoxelAccumulationMode::ADDITIVE:
          case VoxelAccumulationMode::ADDITIVE_WEIGHTED:
            voxel = std::shared_ptr<AdditiveGaussianVoxel>(new AdditiveGaussianVoxel);
            break;
          case VoxelAccumulationMode::MULTIPLICATIVE:
            voxel = std::shared_ptr<MultiplicativeGaussianVoxel>(new MultiplicativeGaussianVoxel);
            break;
        }
        found = voxels_.insert(found, std::make_pair(coord, voxel));
      }

      auto& voxel = found->second;
      voxel->append(cloud.at(i).getVector4fMap().template cast<double>(), covs[i]);
    }

    for(auto& voxel : voxels_) {
      voxel.second->finalize();
    }
  }

  Eigen::Vector3i voxel_coord(const Eigen::Vector4d& x) const {
    return (x.array() / voxel_resolution_ - 0.5).floor().template cast<int>().template head<3>();
  }

  Eigen::Vector4d voxel_origin(const Eigen::Vector3i& coord) const {
    Eigen::Vector3d origin = (coord.template cast<double>().array() + 0.5) * voxel_resolution_;
    return Eigen::Vector4d(origin[0], origin[1], origin[2], 1.0f);
  }

  GaussianVoxel::Ptr lookup_voxel(const Eigen::Vector3i& coord) const {
    auto found = voxels_.find(coord);
    if(found == voxels_.end()) {
      return nullptr;
    }

    return found->second;
  }

private:
  double voxel_resolution_;
  VoxelAccumulationMode voxel_mode_;

  using VoxelMap = std::unordered_map<Eigen::Vector3i, GaussianVoxel::Ptr, Vector3iHash, std::equal_to<Eigen::Vector3i>, Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, GaussianVoxel::Ptr>>>;
  VoxelMap voxels_;
};

}  // namespace fast_gicp

#endif