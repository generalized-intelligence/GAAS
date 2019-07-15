/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Mar 11, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file operators.hpp
 * @brief File with some helper functions related to matrix/vector operations.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_KINEMATICS_OPERATORS_HPP_
#define INCLUDE_OKVIS_KINEMATICS_OPERATORS_HPP_

#include <stdint.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief kinematics Namespace for kinematics functionality, i.e. transformations and stuff.
namespace kinematics {

// some free helper functions

/// \brief Cross matrix of a vector [x,y,z].
///        Adopted from Schweizer-Messer by Paul Furgale.
/// \tparam Scalar_T The scalar type, auto-deducible (typically double).
/// @param[in] x First vector element.
/// @param[in] y Second vector element.
/// @param[in] z Third vector element.
template<typename Scalar_T>
inline Eigen::Matrix<Scalar_T, 3, 3> crossMx(Scalar_T x, Scalar_T y, Scalar_T z)
{
  Eigen::Matrix<Scalar_T, 3, 3> C;
  C(0, 0) = 0.0;
  C(0, 1) = -z;
  C(0, 2) = y;
  C(1, 0) = z;
  C(1, 1) = 0.0;
  C(1, 2) = -x;
  C(2, 0) = -y;
  C(2, 1) = x;
  C(2, 2) = 0.0;
  return C;
}

/// \brief Cross matrix of a vector v.
///        Adopted from Schweizer-Messer by Paul Furgale.
/// \tparam Derived_T The vector type, auto-deducible.
/// @param[in] v The vector.
template<typename Derived_T>
inline Eigen::Matrix<typename Eigen::internal::traits<Derived_T>::Scalar, 3, 3> crossMx(
    Eigen::MatrixBase<Derived_T> const & v)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_T>, 3);
  assert((v.cols()==3 && v.rows()==1)||(v.rows()==3 && v.cols()==1));
  return crossMx(v(0, 0), v(1, 0), v(2, 0));
}

/// \brief Plus matrix of a quaternion, i.e. q_AB*q_BC = plus(q_AB)*q_BC.coeffs().
/// @param[in] q_AB A Quaternion.
inline Eigen::Matrix4d plus(const Eigen::Quaterniond & q_AB) {
  Eigen::Vector4d q = q_AB.coeffs();
  Eigen::Matrix4d Q;
  Q(0,0) =  q[3]; Q(0,1) = -q[2]; Q(0,2) =  q[1]; Q(0,3) =  q[0];
  Q(1,0) =  q[2]; Q(1,1) =  q[3]; Q(1,2) = -q[0]; Q(1,3) =  q[1];
  Q(2,0) = -q[1]; Q(2,1) =  q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  return Q;
}

/// \brief Oplus matrix of a quaternion, i.e. q_AB*q_BC = oplus(q_BC)*q_AB.coeffs().
/// @param[in] q_BC A Quaternion.
inline Eigen::Matrix4d oplus(const Eigen::Quaterniond & q_BC) {
  Eigen::Vector4d q = q_BC.coeffs();
  Eigen::Matrix4d Q;
  Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
  Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
  Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  return Q;
}

} // namespace kinematics
} // namespace okvis



#endif /* INCLUDE_OKVIS_KINEMATICS_OPERATORS_HPP_ */
