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
 * @file SpeedAndBiasError.hpp
 * @brief Header file for the SpeedAndBiasError class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_SPEEDANDBIASERROR_HPP_
#define INCLUDE_OKVIS_CERES_SPEEDANDBIASERROR_HPP_

#include <vector>
#include <Eigen/Core>
#include "ceres/ceres.h"
#include <okvis/assert_macros.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/ceres/ErrorInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

class SpeedAndBiasError : public ::ceres::SizedCostFunction<
    9 /* number of residuals */,
    9 /* size of first parameter */>,
    public ErrorInterface {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<9, 9> base_t;

  /// \brief Number of residuals (9)
  static const int kNumResiduals = 9;

  /// \brief The information matrix type (9x9).
  typedef Eigen::Matrix<double, 9, 9> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 9, 9> covariance_t;

  /// \brief Default constructor.
  SpeedAndBiasError();

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  SpeedAndBiasError(const okvis::SpeedAndBias & measurement,
                    const information_t & information);

  /// \brief Construct with measurement and variance.
  /// @param[in] measurement The measurement.
  /// @param[in] speedVariance The variance of the speed measurement, i.e. information_ has variance in its diagonal.
  /// @param[in] gyrBiasVariance The variance of the gyro bias measurement, i.e. information_ has variance in its diagonal.
  /// @param[in] accBiasVariance The variance of the accelerometer bias measurement, i.e. information_ has variance in its diagonal.
  SpeedAndBiasError(const okvis::SpeedAndBiases& measurement,
                    double speedVariance, double gyrBiasVariance,
                    double accBiasVariance);

  /// \brief Trivial destructor.
  virtual ~SpeedAndBiasError() {
  }

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const okvis::SpeedAndBias & measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t & information);

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector.
  const okvis::SpeedAndBias& measurement() const {
    return measurement_;
  }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t& information() const {
    return information_;
  }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const covariance_t& covariance() const {
    return covariance_;
  }

  // error term and Jacobian implementation
  /**
   * @brief This evaluates the error term and additionally computes the Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const {
    return kNumResiduals;
  }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const {
    return parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameterBlockId ID of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Residual block type as string
  virtual std::string typeInfo() const {
    return "SpeedAndBiasError";
  }

 protected:

  // the measurement
  okvis::SpeedAndBias measurement_; ///< The (9D) measurement.

  // weighting related
  information_t information_; ///< The 9x9 information matrix.
  information_t squareRootInformation_; ///< The 9x9 square root information matrix.
  covariance_t covariance_; ///< The 9x9 covariance matrix.

};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_SPEEDANDBIASERROR_HPP_ */
