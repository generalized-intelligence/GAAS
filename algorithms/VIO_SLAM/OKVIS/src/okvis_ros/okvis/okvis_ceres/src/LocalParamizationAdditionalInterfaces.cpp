/*
 * LocalParamizationAdditionalInterfaces.cpp
 *
 *  Created on: 27 Jul 2015
 *      Author: sleutene
 */

#include <okvis/ceres/LocalParamizationAdditionalInterfaces.hpp>

namespace okvis {
namespace ceres {

// Verifies the correctness of a inplementation.
bool LocalParamizationAdditionalInterfaces::verify(
    const double* x_raw, double purturbation_magnitude) const
{
  const ::ceres::LocalParameterization* casted =
      dynamic_cast<const ::ceres::LocalParameterization*>(this);
  if (!casted) {
    return false;
  }
  // verify plus/minus
  Eigen::VectorXd x(casted->GlobalSize());
  memcpy(x.data(), x_raw, sizeof(double) * casted->GlobalSize());
  Eigen::VectorXd delta_x(casted->LocalSize());
  Eigen::VectorXd x_plus_delta(casted->GlobalSize());
  Eigen::VectorXd delta_x2(casted->LocalSize());
  delta_x.setRandom();
  delta_x *= purturbation_magnitude;
  casted->Plus(x.data(), delta_x.data(), x_plus_delta.data());
  this->Minus(x.data(), x_plus_delta.data(), delta_x2.data());
  if ((delta_x2 - delta_x).norm() > 1.0e-12) {
    return false;
  }

  // plusJacobian numDiff
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus_num_diff(
      casted->GlobalSize(), casted->LocalSize());
  const double dx = 1.0e-9;
  for (int i = 0; i < casted->LocalSize(); ++i) {
    Eigen::VectorXd delta_p(casted->LocalSize());
    delta_p.setZero();
    delta_p[i] = dx;
    Eigen::VectorXd delta_m(casted->LocalSize());
    delta_m.setZero();
    delta_m[i] = -dx;

    // reset
    Eigen::VectorXd x_p(casted->GlobalSize());
    Eigen::VectorXd x_m(casted->GlobalSize());
    memcpy(x_p.data(), x_raw, sizeof(double) * casted->GlobalSize());
    memcpy(x_m.data(), x_raw, sizeof(double) * casted->GlobalSize());
    casted->Plus(x.data(), delta_p.data(), x_p.data());
    casted->Plus(x.data(), delta_m.data(), x_m.data());
    J_plus_num_diff.col(i) = (x_p - x_m) / (2 * dx);
  }

  // verify lift
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus(casted->GlobalSize(),
                                                        casted->LocalSize());
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_lift(casted->LocalSize(),
                                                        casted->GlobalSize());
  casted->ComputeJacobian(x_raw, J_plus.data());
  ComputeLiftJacobian(x_raw, J_lift.data());
  Eigen::MatrixXd identity(casted->LocalSize(), casted->LocalSize());
  identity.setIdentity();
  if (((J_lift * J_plus) - identity).norm() > 1.0e-6) {
    return false;
  }

  // verify numDiff jacobian
  if ((J_plus - J_plus_num_diff).norm() > 1.0e-6) {
    return false;
  }

  // everything fine...
  return true;
}

} // namespace ceres
} // namespace okvis

