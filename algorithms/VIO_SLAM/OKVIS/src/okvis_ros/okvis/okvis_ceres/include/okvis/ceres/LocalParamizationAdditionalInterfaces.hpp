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
 *  Created on: Feb 2, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file LocalParamizationAdditionalInterfaces.hpp
 * @brief Header file for the LocalParamizationAdditionalInterfaces class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_LOCALPARAMIZATIONADDITIONALINTERFACES_HPP_
#define INCLUDE_OKVIS_CERES_LOCALPARAMIZATIONADDITIONALINTERFACES_HPP_

#include "ceres/ceres.h"
#include <okvis/assert_macros.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief Provides some additional interfaces to ceres' LocalParamization
///        than are needed in the generic marginalisation okvis::ceres::MarginalizationError.
class LocalParamizationAdditionalInterfaces {
 public:

  /// \brief Trivial destructor.
  virtual ~LocalParamizationAdditionalInterfaces() {
  }

  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const = 0;

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const = 0;

  /// \brief Verifies the correctness of an inplementation by means of numeric Jacobians.
  /// @param[in] x_raw Linearisation point of the variable.
  /// @param[in] purturbation_magnitude Magnitude of the delta used for numeric Jacobians.
  /// \return True on success.
  virtual bool verify(const double* x_raw, double purturbation_magnitude = 1.0e-6) const ;
};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_LOCALPARAMIZATIONADDITIONALINTERFACES_HPP_ */
