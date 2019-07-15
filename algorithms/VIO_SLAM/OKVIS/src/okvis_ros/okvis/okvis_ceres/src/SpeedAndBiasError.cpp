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
 *  Created on: Sep 10, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file SpeedAndBiasError.cpp
 * @brief Source file for the SpeedAndBiasError class.
 * @author Stefan Leutenegger
 */

#include <okvis/ceres/SpeedAndBiasError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement and information matrix
SpeedAndBiasError::SpeedAndBiasError(const okvis::SpeedAndBias & measurement,
                                     const information_t & information) {
  setMeasurement(measurement);
  setInformation(information);
}

// Construct with measurement and variance.
SpeedAndBiasError::SpeedAndBiasError(const okvis::SpeedAndBiases& measurement,
                                     double speedVariance,
                                     double gyrBiasVariance,
                                     double accBiasVariance) {
  setMeasurement(measurement);

  information_t information;
  information.setZero();
  information.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0
      / speedVariance;
  information.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0
      / gyrBiasVariance;
  information.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0
      / accBiasVariance;

  setInformation(information);
}

// Set the information.
void SpeedAndBiasError::setInformation(const information_t & information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
}

// This evaluates the error term and additionally computes the Jacobians.
bool SpeedAndBiasError::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool SpeedAndBiasError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobiansMinimal) const {

  // compute error
  Eigen::Map<const okvis::SpeedAndBias> estimate(parameters[0]);
  okvis::SpeedAndBias error = measurement_ - estimate;

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 9, 1> > weighted_error(residuals);
  weighted_error = squareRootInformation_ * error;

  // compute Jacobian - this is rather trivial in this case...
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > J0(
          jacobians[0]);
      J0 = -squareRootInformation_ * Eigen::Matrix<double, 9, 9>::Identity();
    }
  }
  if (jacobiansMinimal != NULL) {
    if (jacobiansMinimal[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > J0min(
          jacobiansMinimal[0]);
      J0min = -squareRootInformation_ * Eigen::Matrix<double, 9, 9>::Identity();
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
