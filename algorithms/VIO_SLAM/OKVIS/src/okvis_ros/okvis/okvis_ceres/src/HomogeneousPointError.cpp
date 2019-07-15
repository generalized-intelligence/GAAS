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
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file HomogeneousPointError.cpp
 * @brief Source file for the HomogeneousPointError class.
 * @author Stefan Leutenegger
 */

#include <okvis/ceres/HomogeneousPointError.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement and information matrix.
HomogeneousPointError::HomogeneousPointError(
    const Eigen::Vector4d & measurement, const information_t & information) {
  setMeasurement(measurement);
  setInformation(information);
}

// Construct with measurement and variance.
HomogeneousPointError::HomogeneousPointError(
    const Eigen::Vector4d & measurement, double variance) {
  setMeasurement(measurement);
  setInformation(Eigen::Matrix3d::Identity() * 1.0 / variance);
}

// Construct with measurement and variance.
void HomogeneousPointError::setInformation(const information_t & information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  _squareRootInformation = lltOfInformation.matrixL().transpose();
}

// This evaluates the error term and additionally computes the Jacobians.
bool HomogeneousPointError::Evaluate(double const* const * parameters,
                                     double* residuals,
                                     double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool HomogeneousPointError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobiansMinimal) const {

  // compute error
  Eigen::Vector4d hp(parameters[0][0], parameters[0][1], parameters[0][2],
                     parameters[0][3]);
  // delta
  Eigen::Vector3d error;
  HomogeneousPointLocalParameterization::minus(&measurement_[0],
                                               &parameters[0][0], &error[0]);

  //LOG(INFO)<<hp.toHomogeneous().transpose() << " : " << measurement.transpose();

  // weigh it
  Eigen::Map<Eigen::Vector3d> weighted_error(residuals);
  weighted_error = _squareRootInformation * error;

  // compute Jacobian...
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 3, 4, Eigen::RowMajor> J_lift;
      HomogeneousPointLocalParameterization::liftJacobian(parameters[0],
                                                          J_lift.data());
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_plus;
      HomogeneousPointLocalParameterization::plusJacobian(parameters[0],
                                                          J_plus.data());

      Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > J0(
          jacobians[0]);
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0_minimal = J_lift * J_plus;
      J0_minimal = (_squareRootInformation * J0_minimal).eval();

      // hallucinate Jacobian w.r.t. state
      J0 = J0_minimal * J_lift;

      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[0] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > J0_minimal_mapped(
              jacobiansMinimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis

