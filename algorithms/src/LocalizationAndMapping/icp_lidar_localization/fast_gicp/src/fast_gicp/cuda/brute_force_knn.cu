#include <Eigen/Core>

#include <thrust/sequence.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/zip_iterator.h>

#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/priority_queue.h>

namespace fast_gicp {
  namespace cuda {

namespace {
  struct neighborsearch_kernel {
    neighborsearch_kernel(int k, const thrust::device_vector<Eigen::Vector3f>& target, thrust::device_vector<thrust::pair<float, int>>& k_neighbors)
        : k(k), num_target_points(target.size()), target_points_ptr(target.data()), k_neighbors_ptr(k_neighbors.data()) {}

    template<typename Tuple>
    __host__ __device__ void operator()(const Tuple& idx_x) const {
      // threadIdx doesn't work because thrust split for_each in two loops
      int idx = thrust::get<0>(idx_x);
      const Eigen::Vector3f& x = thrust::get<1>(idx_x);

      // target points buffer & nn output buffer
      const Eigen::Vector3f* pts = thrust::raw_pointer_cast(target_points_ptr);
      thrust::pair<float, int>* k_neighbors = thrust::raw_pointer_cast(k_neighbors_ptr) + idx * k;

      // priority queue
      struct compare_type {
        bool operator()(const thrust::pair<float, int>& lhs, const thrust::pair<float, int>& rhs) {
          return lhs.first < rhs.first;
        }
      };

      typedef nvbio::vector_view<thrust::pair<float, int>*> vector_type;
      typedef nvbio::priority_queue<thrust::pair<float, int>, vector_type, compare_type> queue_type;
      queue_type queue(vector_type(0, k_neighbors - 1));

      for(int i = 0; i < k; i++) {
        float sq_dist = (pts[i] - x).squaredNorm();
        queue.push(thrust::make_pair(sq_dist, i));
      }

      for(int i = k; i < num_target_points; i++) {
        float sq_dist = (pts[i] - x).squaredNorm();
        if(sq_dist < queue.top().first) {
          queue.pop();
          queue.push(thrust::make_pair(sq_dist, i));
        }
      }
    }

    const int k;
    const int num_target_points;
    thrust::device_ptr<const Eigen::Vector3f> target_points_ptr;

    thrust::device_ptr<thrust::pair<float, int>> k_neighbors_ptr;
  };

  struct sorting_kernel {
    sorting_kernel(int k, thrust::device_vector<thrust::pair<float, int>>& k_neighbors) : k(k), k_neighbors_ptr(k_neighbors.data()) {}

    __host__ __device__ void operator()(int idx) const {
      // target points buffer & nn output buffer
      thrust::pair<float, int>* k_neighbors = thrust::raw_pointer_cast(k_neighbors_ptr) + idx * k;

      // priority queue
      struct compare_type {
        bool operator()(const thrust::pair<float, int>& lhs, const thrust::pair<float, int>& rhs) {
          return lhs.first < rhs.first;
        }
      };

      typedef nvbio::vector_view<thrust::pair<float, int>*> vector_type;
      typedef nvbio::priority_queue<thrust::pair<float, int>, vector_type, compare_type> queue_type;
      queue_type queue(vector_type(k, k_neighbors - 1));
      queue.m_size = k;

      for(int i = 0; i < k; i++) {
        thrust::pair<float, int> poped = queue.top();
        queue.pop();

        k_neighbors[k - i - 1] = poped;
      }
    }

    const int k;
    thrust::device_ptr<thrust::pair<float, int>> k_neighbors_ptr;
  };
}

void brute_force_knn_search(const thrust::device_vector<Eigen::Vector3f>& source, const thrust::device_vector<Eigen::Vector3f>& target, int k, thrust::device_vector<thrust::pair<float, int>>& k_neighbors, bool do_sort=false) {
  thrust::device_vector<int> d_indices(source.size());
  thrust::sequence(d_indices.begin(), d_indices.end());

  auto first = thrust::make_zip_iterator(thrust::make_tuple(d_indices.begin(), source.begin()));
  auto last = thrust::make_zip_iterator(thrust::make_tuple(d_indices.end(), source.end()));

  // nvbio::priority_queue requires (k + 1) working space
  k_neighbors.resize(source.size() * k, thrust::make_pair(-1.0f, -1));
  thrust::for_each(first, last, neighborsearch_kernel(k, target, k_neighbors));

  if(do_sort) {
    thrust::for_each(d_indices.begin(), d_indices.end(), sorting_kernel(k, k_neighbors));
  }
}

  }
} // namespace fast_gicp
