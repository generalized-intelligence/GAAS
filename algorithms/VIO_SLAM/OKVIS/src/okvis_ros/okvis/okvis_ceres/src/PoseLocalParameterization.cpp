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
 * @file PoseLocalParameterization.cpp
 * @brief Source file for the PoseLocalParameterization class.
 * @author Stefan Leutenegger
 */

#include <okvis/assert_macros.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/kinematics/operators.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Generalization of the addition operation,
//        x_plus_delta = Plus(x, delta)
//        with the condition that Plus(x, 0) = x.
bool PoseLocalParameterization::Plus(const double* x, const double* delta,
                                     double* x_plus_delta) const {
  return plus(x, delta, x_plus_delta);
}

// Generalization of the addition operation,
//        x_plus_delta = Plus(x, delta)
//        with the condition that Plus(x, 0) = x.
bool PoseLocalParameterization::plus(const double* x, const double* delta,
                                     double* x_plus_delta) {

  Eigen::Map<const Eigen::Matrix<double, 6, 1> > delta_(delta);

  // transform to okvis::kinematics framework
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));

  // call oplus operator in okvis::kinematis
  T.oplus(delta_);

  // copy back
  const Eigen::Vector3d r = T.r();
  x_plus_delta[0] = r[0];
  x_plus_delta[1] = r[1];
  x_plus_delta[2] = r[2];
  const Eigen::Vector4d q = T.q().coeffs();
  x_plus_delta[3] = q[0];
  x_plus_delta[4] = q[1];
  x_plus_delta[5] = q[2];
  x_plus_delta[6] = q[3];

  OKVIS_ASSERT_TRUE_DBG(std::runtime_error, T.q().norm()-1.0<1e-15, "damn.");

  return true;
}

// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization::Minus(const double* x,
                                      const double* x_plus_delta,
                                      double* delta) const {
  return minus(x, x_plus_delta, delta);
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization::ComputeLiftJacobian(const double* x,
                                                    double* jacobian) const {
  return liftJacobian(x, jacobian);
}

// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization::minus(const double* x,
                                      const double* x_plus_delta,
                                      double* delta) {

  delta[0] = x_plus_delta[0] - x[0];
  delta[1] = x_plus_delta[1] - x[1];
  delta[2] = x_plus_delta[2] - x[2];
  const Eigen::Quaterniond q_plus_delta_(x_plus_delta[6], x_plus_delta[3],
                                         x_plus_delta[4], x_plus_delta[5]);
  const Eigen::Quaterniond q_(x[6], x[3], x[4], x[5]);
  Eigen::Map<Eigen::Vector3d> delta_q_(&delta[3]);
  delta_q_ = 2 * (q_plus_delta_ * q_.inverse()).coeffs().template head<3>();
  return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization::plusJacobian(const double* x,
                                             double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > Jp(jacobian);
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
  T.oplusJacobian(Jp);

  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization::liftJacobian(const double* x,
                                             double* jacobian) {

  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > J_lift(jacobian);
  const Eigen::Quaterniond q_inv(x[6], -x[3], -x[4], -x[5]);
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();
  Eigen::Matrix4d Qplus = okvis::kinematics::oplus(q_inv);
  Eigen::Matrix<double, 3, 4> Jq_pinv;
  Jq_pinv.bottomRightCorner<3, 1>().setZero();
  Jq_pinv.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 2.0;
  J_lift.bottomRightCorner<3, 4>() = Jq_pinv * Qplus;

  return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization::ComputeJacobian(const double* x,
                                                double* jacobian) const {

  return plusJacobian(x, jacobian);
}

bool PoseLocalParameterization::VerifyJacobianNumDiff(const double* x,
                                                      double* jacobian,
                                                      double* jacobianNumDiff) {
  plusJacobian(x, jacobian);
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > Jp(jacobian);
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > Jpn(
      jacobianNumDiff);
  double dx = 1e-9;
  Eigen::Matrix<double, 7, 1> xp;
  Eigen::Matrix<double, 7, 1> xm;
  for (size_t i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> delta;
    delta.setZero();
    delta[i] = dx;
    Plus(x, delta.data(), xp.data());
    delta[i] = -dx;
    Plus(x, delta.data(), xm.data());
    Jpn.col(i) = (xp - xm) / (2 * dx);
  }
  if ((Jp - Jpn).norm() < 1e-6)
    return true;
  else
    return false;
}

// Generalization of the addition operation,
//        x_plus_delta = Plus(x, delta)
//        with the condition that Plus(x, 0) = x.
bool PoseLocalParameterization3d::Plus(const double* x, const double* delta,
                                       double* x_plus_delta) const {

  Eigen::Matrix<double, 6, 1> delta_;
  delta_.setZero();
  delta_[3] = delta[0];
  delta_[4] = delta[1];
  delta_[5] = delta[2];

  // transform to sm::kinematics framework
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));

  // call oplus operator in sm::kinematis
  T.oplus(delta_);

  // copy back
  x_plus_delta[0] = T.r()[0];
  x_plus_delta[1] = T.r()[1];
  x_plus_delta[2] = T.r()[2];
  x_plus_delta[3] = T.q().coeffs()[0];
  x_plus_delta[4] = T.q().coeffs()[1];
  x_plus_delta[5] = T.q().coeffs()[2];
  x_plus_delta[6] = T.q().coeffs()[3];

  OKVIS_ASSERT_TRUE_DBG(std::runtime_error, T.q().norm() - 1.0 < 1e-15, "damn.");

  return true;
}

// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization3d::Minus(const double* x,
                                        const double* x_plus_delta,
                                        double* delta) const {

  const Eigen::Quaterniond q_plus_delta_(x_plus_delta[6], x_plus_delta[3],
                                         x_plus_delta[4], x_plus_delta[5]);
  const Eigen::Quaterniond q_(x[6], x[3], x[4], x[5]);
  Eigen::Vector3d delta_q_;
  delta_q_ = 2 * (q_plus_delta_ * q_.inverse()).coeffs().template head<3>();

  delta[0] = delta_q_[0];
  delta[1] = delta_q_[1];
  delta[2] = delta_q_[2];

  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization3d::ComputeLiftJacobian(const double* x,
                                                      double* jacobian) const {
  return liftJacobian(x, jacobian);
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization3d::plusJacobian(const double* x,
                                               double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 7, 3, Eigen::RowMajor> > Jp(jacobian);
  Jp.setZero();

  // rotation: dq*q=qoplus(q)*dq=qoplus(q)*1/2*[I_3;0]*dalpha
  Eigen::Matrix4d Qoplus = okvis::kinematics::oplus(
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
  Eigen::Matrix<double, 4, 3> Jq;
  Jq.bottomRightCorner<3, 3>().setZero();
  Jq.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.5;
  Jp.bottomRightCorner<4, 3>() = Qoplus * Jq;

  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization3d::liftJacobian(const double* x,
                                               double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > J_lift(jacobian);
  J_lift.setZero();

  const Eigen::Quaterniond q_inv(x[6], -x[3], -x[4], -x[5]);
  Eigen::Matrix4d Qplus = okvis::kinematics::oplus(q_inv);
  Eigen::Matrix<double, 3, 4> Jq_pinv = Eigen::Matrix<double, 3, 4>::Zero();
  Jq_pinv(0, 0) = 2.0;
  Jq_pinv(1, 1) = 2.0;
  Jq_pinv(2, 2) = 2.0;

  J_lift.bottomRightCorner<3, 4>() = Jq_pinv * Qplus;

  return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization3d::ComputeJacobian(const double* x,
                                                  double* jacobian) const {
  return plusJacobian(x, jacobian);
}

// Generalization of the addition operation,
//        x_plus_delta = Plus(x, delta)
//        with the condition that Plus(x, 0) = x.
bool PoseLocalParameterization4d::Plus(const double* x, const double* delta,
                                       double* x_plus_delta) const {

  Eigen::Matrix<double, 6, 1> delta_;
  delta_.setZero();
  delta_[0] = delta[0];
  delta_[1] = delta[1];
  delta_[2] = delta[2];
  delta_[5] = delta[3];

  // transform to sm::kinematics framework
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));

  // call oplus operator in sm::kinematis
  T.oplus(delta_);

  // copy back
  x_plus_delta[0] = T.r()[0];
  x_plus_delta[1] = T.r()[1];
  x_plus_delta[2] = T.r()[2];
  x_plus_delta[3] = T.q().coeffs()[0];
  x_plus_delta[4] = T.q().coeffs()[1];
  x_plus_delta[5] = T.q().coeffs()[2];
  x_plus_delta[6] = T.q().coeffs()[3];

  OKVIS_ASSERT_TRUE_DBG(std::runtime_error, T.q().norm() - 1.0 < 1e-15, "damn.");

  return true;
}

// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization4d::Minus(const double* x,
                                        const double* x_plus_delta,
                                        double* delta) const {

  delta[0] = x_plus_delta[0] - x[0];
  delta[1] = x_plus_delta[1] - x[1];
  delta[2] = x_plus_delta[2] - x[2];
  const Eigen::Quaterniond q_plus_delta_(x_plus_delta[6], x_plus_delta[3],
                                         x_plus_delta[4], x_plus_delta[5]);
  const Eigen::Quaterniond q_(x[6], x[3], x[4], x[5]);
  delta[3] = 2 * (q_plus_delta_ * q_.inverse()).coeffs()[2];

  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization4d::ComputeLiftJacobian(const double* x,
                                                      double* jacobian) const {
  return liftJacobian(x, jacobian);
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization4d::plusJacobian(const double* x,
                                               double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 7, 4, Eigen::RowMajor> > Jp(jacobian);
  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> Jp_full;
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
  T.oplusJacobian(Jp_full);
  Jp.topLeftCorner<7,3>()=Jp_full.topLeftCorner<7,3>();
  Jp.bottomRightCorner<7,1>()=Jp_full.bottomRightCorner<7,1>();
  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization4d::liftJacobian(const double* x,
                                               double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor> > J_lift(jacobian);
  const Eigen::Quaterniond q_inv(x[6], -x[3], -x[4], -x[5]);
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();
  Eigen::Matrix4d Qplus = okvis::kinematics::oplus(q_inv);
  Eigen::Matrix<double, 3, 4> Jq_pinv;
  Jq_pinv.bottomRightCorner<3, 1>().setZero();
  Jq_pinv.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 2.0;
  J_lift.bottomRightCorner<1, 4>() = (Jq_pinv * Qplus).bottomRightCorner<1, 4>();

  return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization4d::ComputeJacobian(const double* x,
                                                  double* jacobian) const {
  return plusJacobian(x, jacobian);
}

// Generalization of the addition operation,
//        x_plus_delta = Plus(x, delta)
//        with the condition that Plus(x, 0) = x.
bool PoseLocalParameterization2d::Plus(const double* x, const double* delta,
                                       double* x_plus_delta) const {

  Eigen::Matrix<double, 6, 1> delta_;
  delta_.setZero();
  delta_[3] = delta[0];
  delta_[4] = delta[1];

  // transform to okvis::kinematics framework
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));

  // call oplus operator in okvis::kinematis
  T.oplus(delta_);

  // copy back
  x_plus_delta[0] = T.r()[0];
  x_plus_delta[1] = T.r()[1];
  x_plus_delta[2] = T.r()[2];
  x_plus_delta[3] = T.q().coeffs()[0];
  x_plus_delta[4] = T.q().coeffs()[1];
  x_plus_delta[5] = T.q().coeffs()[2];
  x_plus_delta[6] = T.q().coeffs()[3];

  OKVIS_ASSERT_TRUE_DBG(std::runtime_error, T.q().norm() - 1.0 < 1e-15, "damn.");

  return true;
}

// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization2d::Minus(const double* x,
                                        const double* x_plus_delta,
                                        double* delta) const {

  const Eigen::Quaterniond q_plus_delta_(x_plus_delta[6], x_plus_delta[3],
                                         x_plus_delta[4], x_plus_delta[5]);
  const Eigen::Quaterniond q_(x[6], x[3], x[4], x[5]);
  Eigen::Vector3d delta_q_;
  delta_q_ = 2 * (q_plus_delta_ * q_.inverse()).coeffs().template head<3>();

  delta[0] = delta_q_[0];
  delta[1] = delta_q_[1];

  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization2d::ComputeLiftJacobian(const double* x,
                                                      double* jacobian) const {
  return liftJacobian(x, jacobian);
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization2d::plusJacobian(const double* x,
                                               double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 7, 2, Eigen::RowMajor> > Jp(jacobian);
  Jp.setZero();

  // rotation: dq*q=qoplus(q)*dq=qoplus(q)*1/2*[I_3;0]*dalpha
  Eigen::Matrix4d Qoplus = okvis::kinematics::oplus(
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
  Eigen::Matrix<double, 4, 2> Jq;
  Jq.bottomRightCorner<2, 2>().setZero();
  Jq.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() * 0.5;
  Jp.bottomRightCorner<4, 2>() = Qoplus * Jq;

  return true;
}

// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization2d::liftJacobian(const double* x,
                                               double* jacobian) {
  Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_lift(jacobian);
  J_lift.setZero();

  const Eigen::Quaterniond q_inv(x[6], -x[3], -x[4], -x[5]);
  Eigen::Matrix4d Qplus = okvis::kinematics::oplus(q_inv);
  Eigen::Matrix<double, 2, 4> Jq_pinv = Eigen::Matrix<double, 2, 4>::Zero();
  Jq_pinv(0, 0) = 2.0;
  Jq_pinv(1, 1) = 2.0;

  J_lift.bottomRightCorner<2, 4>() = Jq_pinv * Qplus;

  return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization2d::ComputeJacobian(const double* x,
                                                  double* jacobian) const {
  return plusJacobian(x, jacobian);
}

}  // namespace ceres
}  // namespace okvis
