#ifndef YGZ_NUM_TYPES_H
#define YGZ_NUM_TYPES_H

// for Eigen
#include <Eigen/Core>
#include <Eigen/Dense>  // linear algebra
#include <Eigen/StdVector>

#include "so3.hpp"
#include "se3.hpp"

#include <memory>
#include <mutex>
#include <thread>

using Sophus::SO3f;
using Sophus::SE3f;
using Sophus::SO3d;
using Sophus::SE3d;

namespace ygz {
    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Vector2f;
    using Eigen::Vector3f;
    using Eigen::Vector4f;

    using Eigen::Matrix2f;
    using Eigen::Matrix3f;
    using Eigen::Matrix4f;
    using Eigen::Matrix2d;
    using Eigen::Matrix3d;
    using Eigen::Matrix4d;
    using Eigen::Quaternionf;

    // other things I need in optimiztion
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<float, 6, 1> Vector6f;

    typedef Eigen::Matrix<double, 9, 1> Vector9d;
    typedef Eigen::Matrix<float, 9, 1> Vector9f;

    typedef Eigen::Matrix<float, 15, 15> Matrix15f;
    typedef Eigen::Matrix<double, 15, 15> Matrix15d;

    typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;

    typedef Eigen::Matrix<double, 9, 9> Matrix9d;
    typedef Eigen::Matrix<float, 9, 9> Matrix9f;

    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<float, 6, 6> Matrix6f;

    typedef Eigen::Matrix<double, 2, 6> Matrix26d;
    typedef Eigen::Matrix<float, 2, 6> Matrix26f;

    typedef unsigned char uchar;

    // vector of Eigen things
    typedef std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
    typedef std::vector<Vector3f, Eigen::aligned_allocator<Vector3f>> VecVector3f;
    typedef std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
    typedef std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>> VecVector2f;

    typedef std::vector<SE3f, Eigen::aligned_allocator<SE3f>> VecSE3f;
    typedef std::vector<SE3d, Eigen::aligned_allocator<SE3d>> VecSE3d;
    typedef std::vector<SO3f, Eigen::aligned_allocator<SO3f>> VecSO3f;
    typedef std::vector<SO3d, Eigen::aligned_allocator<SO3d>> VecSO3d;

}

#endif
