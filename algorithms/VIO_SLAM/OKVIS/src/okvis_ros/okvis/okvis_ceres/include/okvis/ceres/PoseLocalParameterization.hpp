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
 * @file PoseLocalParameterization.hpp
 * @brief Header file for the PoseLocalParemerization class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_POSELOCALPARAMETERIZATION_HPP_
#define INCLUDE_OKVIS_CERES_POSELOCALPARAMETERIZATION_HPP_

#include "ceres/ceres.h"
#include <okvis/assert_macros.hpp>
#include <okvis/ceres/LocalParamizationAdditionalInterfaces.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
class PoseLocalParameterization : public ::ceres::LocalParameterization,
    public LocalParamizationAdditionalInterfaces {
 public:

  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization() {
  }

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const;

  // provide these as static for easy use elsewhere:

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  static bool plus(const double* x, const double* delta, double* x_plus_delta);

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double* x, double* jacobian);

  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  static bool minus(const double* x, const double* x_plus_delta, double* delta);

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double* x, double* jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const {
    return 7;
  }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const {
    return 6;
  }

  // added convenient check
  bool VerifyJacobianNumDiff(const double* x, double* jacobian,
                             double* jacobianNumDiff);
};

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
///        Here, we only perturb the translation though.
class PoseLocalParameterization3d : public ::ceres::LocalParameterization,
    public LocalParamizationAdditionalInterfaces {
 public:

  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization3d() {
  }

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const;

  // provide these as static for easy use elsewhere:
  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double* x, double* jacobian);

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double* x, double* jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const {
    return 7;
  }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const {
    return 3;
  }
};

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
///        Here, we only perturb the translation and yaw though.
class PoseLocalParameterization4d : public ::ceres::LocalParameterization,
    public LocalParamizationAdditionalInterfaces {
 public:

  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization4d() {
  }

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const;

  // provide these as static for easy use elsewhere:
  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double* x, double* jacobian);

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double* x, double* jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const {
    return 7;
  }

  /// \brief The parameter block local dimension.
  virtual int LocalSize() const {
    return 4;
  }
};

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
///        Here, we only perturb roll and pitch, i.e. dalpha = [dalpha1, dalpha2, 0]^T.
class PoseLocalParameterization2d : public ::ceres::LocalParameterization,
    public LocalParamizationAdditionalInterfaces {
 public:

  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization2d() {
  }

  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x Variable.
  /// @param[in] delta Perturbation.
  /// @param[out] x_plus_delta Perturbed x.
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;

  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x Variable.
  /// @param[in] x_plus_delta Perturbed variable.
  /// @param[out] delta minimal difference.
  /// \return True on success.
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const;

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const;

  // provide these as static for easy use elsewhere:
  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x Variable.
  /// @param[out] jacobian The Jacobian.
  static bool plusJacobian(const double* x, double* jacobian);

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  static bool liftJacobian(const double* x, double* jacobian);

  /// \brief The parameter block dimension.
  virtual int GlobalSize() const {
    return 7;
  }
  /// \brief The parameter block local dimension.
  virtual int LocalSize() const {
    return 2;
  }
};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_POSELOCALPARAMETERIZATION_HPP_ */
