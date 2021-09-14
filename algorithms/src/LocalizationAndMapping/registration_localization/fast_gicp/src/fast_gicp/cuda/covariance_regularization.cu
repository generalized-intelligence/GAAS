#include <fast_gicp/cuda/covariance_regularization.cuh>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <thrust/copy.h>
#include <thrust/iterator/transform_output_iterator.h>

namespace fast_gicp {
namespace cuda {

namespace {

struct svd_kernel {
  __host__ __device__ thrust::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Matrix3f> operator()(const thrust::tuple<Eigen::Vector3f, Eigen::Matrix3f>& input) const {
    const auto& mean = thrust::get<0>(input);
    const auto& cov = thrust::get<1>(input);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    eig.computeDirect(cov);

    return thrust::make_tuple(mean, eig.eigenvalues(), eig.eigenvectors());
  }
};

struct eigenvalue_filter_kernel {
  __host__ __device__ bool operator()(const thrust::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Matrix3f>& input) const {
    const auto& values = thrust::get<1>(input);
    return values[1] > values[2] * 0.1;
  }
};

struct svd_reconstruction_kernel {
  __host__ __device__ thrust::tuple<Eigen::Vector3f, Eigen::Matrix3f> operator()(const thrust::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Matrix3f>& input) const {
    const auto& mean = thrust::get<0>(input);
    const auto& values = thrust::get<1>(input);
    const auto& vecs = thrust::get<2>(input);

    Eigen::Matrix3f values_diag = Eigen::Vector3f(1e-3f, 1.0f, 1.0f).asDiagonal();
    Eigen::Matrix3f vecs_inv = vecs.inverse();
    return thrust::make_tuple(mean, (vecs * values_diag * vecs_inv).eval());
  }
};

struct covariance_regularization_svd {
  __host__ __device__ void operator()(Eigen::Matrix3f& cov) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    eig.computeDirect(cov);

    // why this doen't work...???
    // cov = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().inverse();
    Eigen::Matrix3f values = Eigen::Vector3f(1e-3, 1, 1).asDiagonal();
    Eigen::Matrix3f v_inv = eig.eigenvectors().inverse();
    cov = eig.eigenvectors() * values * v_inv;

    // JacobiSVD is not supported on CUDA
    // Eigen::JacobiSVD(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Eigen::Vector3f values(1, 1, 1e-3);
    // cov = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
  }
};

struct covariance_regularization_frobenius {
  __host__ __device__ void operator()(Eigen::Matrix3f& cov) const {
    float lambda = 1e-3;
    Eigen::Matrix3f C = cov + lambda * Eigen::Matrix3f::Identity();
    Eigen::Matrix3f C_inv = C.inverse();
    Eigen::Matrix3f C_norm = (C_inv / C_inv.norm()).inverse();
    cov = C_norm;
  }
};

struct covariance_regularization_mineig {
  __host__ __device__ void operator()(Eigen::Matrix3f& cov) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    eig.computeDirect(cov);

    Eigen::Vector3f values = eig.eigenvalues();
    for (int i = 0; i < 3; i++) {
      values[i] = fmaxf(1e-3f, values[i]);
    }

    Eigen::Matrix3f v_diag = values.asDiagonal();
    Eigen::Matrix3f v_inv = eig.eigenvectors().inverse();
    cov = eig.eigenvectors() * v_diag * v_inv;
  }
};

}  // namespace

void covariance_regularization(thrust::device_vector<Eigen::Vector3f>& means, thrust::device_vector<Eigen::Matrix3f>& covs, RegularizationMethod method) {
  if (method == RegularizationMethod::PLANE) {
    // thrust::for_each(covs.begin(), covs.end(), covariance_regularization_svd());
    // return;

    thrust::device_vector<Eigen::Vector3f> means_(means.size());
    thrust::device_vector<Eigen::Matrix3f> covs_(covs.size());

    auto first = thrust::make_transform_iterator(thrust::make_zip_iterator(thrust::make_tuple(means.begin(), covs.begin())), svd_kernel());
    auto last = thrust::make_transform_iterator(thrust::make_zip_iterator(thrust::make_tuple(means.end(), covs.end())), svd_kernel());
    auto output_first = thrust::make_transform_output_iterator(thrust::make_zip_iterator(thrust::make_tuple(means_.begin(), covs_.begin())), svd_reconstruction_kernel());
    auto output_last = thrust::copy_if(first, last, output_first, eigenvalue_filter_kernel());

    int num_valid_points = output_last - output_first;
    means_.erase(means_.begin() + num_valid_points, means_.end());
    covs_.erase(covs_.begin() + num_valid_points, covs_.end());

    means = std::move(means_);
    covs = std::move(covs_);
  } else if (method == RegularizationMethod::FROBENIUS) {
    thrust::for_each(covs.begin(), covs.end(), covariance_regularization_frobenius());
  } else if (method == RegularizationMethod::MIN_EIG) {
    thrust::for_each(covs.begin(), covs.end(), covariance_regularization_mineig());
  } else {
    std::cerr << "unimplemented covariance regularization method was selected!!" << std::endl;
  }
}

}  // namespace cuda
}  // namespace fast_gicp