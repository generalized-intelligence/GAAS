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
 *  Created on: Mar 10, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file stereo_triangulation.cpp
 * @brief Implementation of the triangulateFast function.
 * @author Stefan Leutenegger
 */

#include <okvis/triangulation/stereo_triangulation.hpp>
#include <okvis/kinematics/operators.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <iostream>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief triangulation A namespace for operations related to triangulation.
namespace triangulation {

// Triangulate the intersection of two rays.
Eigen::Vector4d triangulateFast(const Eigen::Vector3d& p1,
                                const Eigen::Vector3d& e1,
                                const Eigen::Vector3d& p2,
                                const Eigen::Vector3d& e2, double sigma,
                                bool& isValid, bool& isParallel) {
  isParallel = false; // This should be the default.
  // But parallel and invalid is not the same. Points at infinity are valid and parallel.
  isValid = false; // hopefully this will be reset to true.

  // stolen and adapted from the Kneip toolchain geometric_vision/include/geometric_vision/triangulation/impl/triangulation.hpp
  Eigen::Vector3d t12 = p2 - p1;

  // check parallel
  /*if (t12.dot(e1) - t12.dot(e2) < 1.0e-12) {
    if ((e1.cross(e2)).norm() < 6 * sigma) {
      isValid = true;  // check parallel
      isParallel = true;
      return (Eigen::Vector4d((e1[0] + e2[0]) / 2.0, (e1[1] + e2[1]) / 2.0,
                              (e1[2] + e2[2]) / 2.0, 1e-2).normalized());
    }
  }*/

  Eigen::Vector2d b;
  b[0] = t12.dot(e1);
  b[1] = t12.dot(e2);
  Eigen::Matrix2d A;
  A(0, 0) = e1.dot(e1);
  A(1, 0) = e1.dot(e2);
  A(0, 1) = -A(1, 0);
  A(1, 1) = -e2.dot(e2);

  if (A(1, 0) < 0.0) {
    A(1, 0) = -A(1, 0);
    A(0, 1) = -A(0, 1);
    // wrong viewing direction
  };

  bool invertible;
  Eigen::Matrix2d A_inverse;
  A.computeInverseWithCheck(A_inverse, invertible, 1.0e-6);
  Eigen::Vector2d lambda = A_inverse * b;
  if (!invertible) {
    isParallel = true; // let's note this.
    // parallel. that's fine. but A is not invertible. so handle it separately.
    if ((e1.cross(e2)).norm() < 6 * sigma){
       isValid = true;  // check parallel
    }
    return (Eigen::Vector4d((e1[0] + e2[0]) / 2.0, (e1[1] + e2[1]) / 2.0,
                            (e1[2] + e2[2]) / 2.0, 1e-3).normalized());
  }

  Eigen::Vector3d xm = lambda[0] * e1 + p1;
  Eigen::Vector3d xn = lambda[1] * e2 + p2;
  Eigen::Vector3d midpoint = (xm + xn) / 2.0;

  // check it
  Eigen::Vector3d error = midpoint - xm;
  Eigen::Vector3d diff = midpoint - (p1 + 0.5 * t12);
  const double diff_sq = diff.dot(diff);
  const double chi2 = error.dot(error) * (1.0 / (diff_sq * sigma * sigma));

  isValid = true;
  if (chi2 > 9) {
    isValid = false;  // reject large chi2-errors
  }

  // flip if necessary
  if (diff.dot(e1) < 0) {
    midpoint = (p1 + 0.5 * t12) - diff;
  }

  return Eigen::Vector4d(midpoint[0], midpoint[1], midpoint[2], 1.0).normalized();
}

}

}

