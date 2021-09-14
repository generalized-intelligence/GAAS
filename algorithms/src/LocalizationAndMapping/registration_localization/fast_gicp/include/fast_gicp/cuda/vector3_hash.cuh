#ifndef FAST_GICP_CUDA_VECTOR3_HASH_CUH
#define FAST_GICP_CUDA_VECTOR3_HASH_CUH

namespace fast_gicp {
  namespace cuda {

// taken from boost/hash.hpp
__host__ __device__  inline void hash_combine(uint64_t& h, uint64_t k) {
  const uint64_t m = UINT64_C(0xc6a4a7935bd1e995);
  const int r = 47;

  k *= m;
  k ^= k >> r;
  k *= m;

  h ^= k;
  h *= m;

  h += 0xe6546b64;
}

inline __host__ __device__ bool equal(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) {
  return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
}

// compute vector3i hash
__host__ __device__ inline uint64_t vector3i_hash(const Eigen::Vector3i& x) {
  uint64_t seed = 0;
  hash_combine(seed, x[0]);
  hash_combine(seed, x[1]);
  hash_combine(seed, x[2]);
  return seed;
}

__host__ __device__ inline Eigen::Vector3i calc_voxel_coord(const Eigen::Vector3f& x, float resolution) {
  Eigen::Vector3i coord = (x.array() / resolution - 0.5).floor().cast<int>();
  return coord;
}

  }
}  // namespace fast_gicp

#endif