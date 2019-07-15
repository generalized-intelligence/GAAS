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
 *  Created on: Aug 30, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file SpeedAndBiasParameterBlock.hpp
 * @brief Header file for the SpeedAndBiasParameterBlock class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_SPEEDANDBIASPARAMETERBLOCK_HPP_
#define INCLUDE_OKVIS_CERES_SPEEDANDBIASPARAMETERBLOCK_HPP_

#include <okvis/ceres/ParameterBlockSized.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <Eigen/Core>
#include <okvis/Time.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;

/// \brief Wraps the parameter block for a speed / IMU biases estimate
class SpeedAndBiasParameterBlock :
    public ParameterBlockSized<9, 9, SpeedAndBias> {
 public:

  /// \brief The base class type.
  typedef ParameterBlockSized<9, 9, SpeedAndBias> base_t;

  /// \brief The estimate type (9D vector).
  typedef SpeedAndBias estimate_t;

  /// \brief Default constructor (assumes not fixed).
  SpeedAndBiasParameterBlock();

  /// \brief Constructor with estimate and time.
  /// @param[in] speedAndBias The speed and bias estimate.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  SpeedAndBiasParameterBlock(const SpeedAndBias& speedAndBias, uint64_t id,
                             const okvis::Time& timestamp);

  /// \brief Trivial destructor.
  virtual ~SpeedAndBiasParameterBlock();

  // setters
  /// @brief Set estimate of this parameter block.
  /// @param[in] speedAndBias The estimate to set this to.
  virtual void setEstimate(const SpeedAndBias& speedAndBias);

  /// \brief Set the time.
  /// @param[in] timestamp The timestamp of this state.
  void setTimestamp(const okvis::Time& timestamp) {
    timestamp_ = timestamp;
  }

  // getters
  /// @brief Get estimate.
  /// \return The estimate.
  virtual SpeedAndBias estimate() const;

  /// \brief Get the time.
  /// \return The timestamp of this state.
  okvis::Time timestamp() const {
    return timestamp_;
  }

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const {
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > x0_(x0);
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > Delta_Chi_(Delta_Chi);
    Eigen::Map<Eigen::Matrix<double, 9, 1> > x0_plus_Delta_(x0_plus_Delta);
    x0_plus_Delta_ = x0_ + Delta_Chi_;
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
//  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double* /*unused: x*/,
                            double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > identity(
        jacobian);
    identity.setIdentity();
  }

  // Delta_Chi=x0_plus_Delta[-]x0
  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  /// \return True on success.
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const {
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > x0_(x0);
    Eigen::Map<Eigen::Matrix<double, 9, 1> > Delta_Chi_(Delta_Chi);
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > x0_plus_Delta_(
        x0_plus_Delta);
    Delta_Chi_ = x0_plus_Delta_ - x0_;
  }

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
//  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double* /*unused: x*/,
                            double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > identity(
        jacobian);
    identity.setIdentity();
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const {
    return "SpeedAndBiasParameterBlock";
  }

 private:
  okvis::Time timestamp_; ///< Time of this state.
};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_SPEEDANDBIASPARAMETERBLOCK_HPP_ */
