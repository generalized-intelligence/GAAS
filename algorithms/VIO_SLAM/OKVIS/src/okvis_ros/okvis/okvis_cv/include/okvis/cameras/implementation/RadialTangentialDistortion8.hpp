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
 *  Created on: Jul 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */

#include <Eigen/LU>
#include <iostream>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion8::RadialTangentialDistortion8()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0),
      k3_(0.0),
      k4_(0.0),
      k5_(0.0),
      k6_(0.0)
{
  parameters_.setZero();
}

// Constructor initialising ki
RadialTangentialDistortion8::RadialTangentialDistortion8(double k1, double k2,
                                                         double p1, double p2,
                                                         double k3, double k4,
                                                         double k5, double k6)
{
  parameters_[0] = k1;
  parameters_[1] = k2;
  parameters_[2] = p1;
  parameters_[3] = p2;
  parameters_[4] = k3;
  parameters_[5] = k4;
  parameters_[6] = k5;
  parameters_[7] = k6;
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
  k3_ = k3;
  k4_ = k4;
  k5_ = k5;
  k6_ = k6;
}

bool RadialTangentialDistortion8::setParameters(
    const Eigen::VectorXd & parameters)
{
  if (parameters.cols() != NumDistortionIntrinsics) {
    return false;
  }
  parameters_ = parameters;
  k1_ = parameters[0];
  k2_ = parameters[1];
  p1_ = parameters[2];
  p2_ = parameters[3];
  k3_ = parameters[4];
  k4_ = parameters[5];
  k5_ = parameters[6];
  k6_ = parameters[7];
  return true;
}

bool RadialTangentialDistortion8::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const
{
  // just compute the distorted point
  const double u0 = pointUndistorted[0];
  const double u1 = pointUndistorted[1];
  const double mx_u = u0 * u0;
  const double my_u = u1 * u1;
  const double rho_u = mx_u + my_u;
  if(rho_u>9.0)
    return false; // to avoid confusion of this model
  const double mxy_u = u0 * u1;
  const double rad_dist_u = (1.0 + ((k3_ * rho_u + k2_) * rho_u + k1_) * rho_u)
      / (1.0 + ((k6_ * rho_u + k5_) * rho_u + k4_) * rho_u);
  (*pointDistorted)[0] = u0 * rad_dist_u + 2.0 * p1_ * mxy_u
      + p2_ * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 * rad_dist_u + 2.0 * p2_ * mxy_u
      + p1_ * (rho_u + 2.0 * my_u);
  return true;
}

bool RadialTangentialDistortion8::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian, Eigen::Matrix2Xd * parameterJacobian) const
{
  // first compute the distorted point
  const double u0 = pointUndistorted[0];
  const double u1 = pointUndistorted[1];
  const double mx_u = u0 * u0;
  const double my_u = u1 * u1;
  const double mxy_u = u0 * u1;
  const double rho_u = mx_u + my_u;

  if(rho_u>9.0)
      return false; // to avoid confusion of this model

  const double c = rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0;
  const double c2 = c*c;

  const double rad_dist_u = (1.0 + ((k3_ * rho_u + k2_) * rho_u + k1_) * rho_u)
      / (1.0 + ((k6_ * rho_u + k5_) * rho_u + k4_) * rho_u);
  (*pointDistorted)[0] = u0 * rad_dist_u + 2.0 * p1_ * mxy_u
      + p2_ * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 * rad_dist_u + 2.0 * p2_ * mxy_u
      + p1_ * (rho_u + 2.0 * my_u);

  // next the Jacobian w.r.t. changes on the undistorted point
  Eigen::Matrix2d & J = *pointJacobian;
  J(0,0) = p1_*u1*2.0+p2_*u0*6.0+(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)+(u0*(rho_u*(u0*(k2_+k3_*rho_u)*2.0+k3_*u0*rho_u*2.0)+u0*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u0*(rho_u*(u0*(k5_+k6_*rho_u)*2.0+k6_*u0*rho_u*2.0)+u0*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
  J(0,1) = p1_*u0*2.0+p2_*u1*2.0+(u0*(rho_u*(u1*(k2_+k3_*rho_u)*2.0+k3_*u1*rho_u*2.0)+u1*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u0*(rho_u*(u1*(k5_+k6_*rho_u)*2.0+k6_*u1*rho_u*2.0)+u1*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
  J(1,0) = p1_*u0*2.0+p2_*u1*2.0+(u1*(rho_u*(u0*(k2_+k3_*rho_u)*2.0+k3_*u0*rho_u*2.0)+u0*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u1*(rho_u*(u0*(k5_+k6_*rho_u)*2.0+k6_*u0*rho_u*2.0)+u0*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
  J(1,1) = p1_*u1*6.0+p2_*u0*2.0+(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)+(u1*(rho_u*(u1*(k2_+k3_*rho_u)*2.0+k3_*u1*rho_u*2.0)+u1*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u1*(rho_u*(u1*(k5_+k6_*rho_u)*2.0+k6_*u1*rho_u*2.0)+u1*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;

  if (parameterJacobian) {
    // the Jacobian w.r.t. intrinsics parameters
    const double rho_u2 = rho_u*rho_u;
    const double rho_u3 = rho_u*rho_u2;
    Eigen::Matrix2Xd & Jp = *parameterJacobian;
    Jp.resize(2, NumDistortionIntrinsics);
    Jp(0,0) = (u0*rho_u)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);
    Jp(0,1) = (u0*rho_u2)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);
    Jp(0,2) = u0*u1*2.0;
    Jp(0,3) = (u0*u0)*3.0+u1*u1;
    Jp(0,4) = (u0*rho_u3)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);
    Jp(0,5) = -u0*rho_u*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
    Jp(0,6) = -u0*rho_u2*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
    Jp(0,7) = -u0*rho_u3*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
    Jp(1,0) = (u1*rho_u)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);
    Jp(1,1) = (u1*rho_u2)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);
    Jp(1,2) = u0*u0+(u1*u1)*3.0;
    Jp(1,3) = u0*u1*2.0;
    Jp(1,4) = (u1*rho_u3)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);
    Jp(1,5) = -u1*rho_u*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
    Jp(1,6) = -u1*rho_u2*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
    Jp(1,7) = -u1*rho_u3*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;
  }
  return true;
}

bool RadialTangentialDistortion8::distortWithExternalParameters(
    const Eigen::Vector2d & pointUndistorted,
    const Eigen::VectorXd & parameters, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian, Eigen::Matrix2Xd * parameterJacobian) const
{
  const double k1 = parameters[0];
  const double k2 = parameters[1];
  const double p1 = parameters[2];
  const double p2 = parameters[3];
  const double k3 = parameters[4];
  const double k4 = parameters[5];
  const double k5 = parameters[6];
  const double k6 = parameters[7];
  // first compute the distorted point
  const double u0 = pointUndistorted[0];
    const double u1 = pointUndistorted[1];
    const double mx_u = u0 * u0;
    const double my_u = u1 * u1;
    const double mxy_u = u0 * u1;
    const double rho_u = mx_u + my_u;

    if(rho_u>9.0)
          return false; // to avoid confusion of this model

    const double c = rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0;
    const double c2 = c*c;

    const double rad_dist_u = (1.0 + ((k3 * rho_u + k2) * rho_u + k1) * rho_u)
          / (1.0 + ((k6 * rho_u + k5) * rho_u + k4) * rho_u);
      (*pointDistorted)[0] = u0 * rad_dist_u + 2.0 * p1 * mxy_u
          + p2 * (rho_u + 2.0 * mx_u);
      (*pointDistorted)[1] = u1 * rad_dist_u + 2.0 * p2 * mxy_u
          + p1 * (rho_u + 2.0 * my_u);

      // next the Jacobian w.r.t. changes on the undistorted point
      Eigen::Matrix2d & J = *pointJacobian;
      J(0,0) = p1*u1*2.0+p2*u0*6.0+(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)+(u0*(rho_u*(u0*(k2+k3*rho_u)*2.0+k3*u0*rho_u*2.0)+u0*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u0*(rho_u*(u0*(k5+k6*rho_u)*2.0+k6*u0*rho_u*2.0)+u0*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
      J(0,1) = p1*u0*2.0+p2*u1*2.0+(u0*(rho_u*(u1*(k2+k3*rho_u)*2.0+k3*u1*rho_u*2.0)+u1*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u0*(rho_u*(u1*(k5+k6*rho_u)*2.0+k6*u1*rho_u*2.0)+u1*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
      J(1,0) = p1*u0*2.0+p2*u1*2.0+(u1*(rho_u*(u0*(k2+k3*rho_u)*2.0+k3*u0*rho_u*2.0)+u0*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u1*(rho_u*(u0*(k5+k6*rho_u)*2.0+k6*u0*rho_u*2.0)+u0*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
      J(1,1) = p1*u1*6.0+p2*u0*2.0+(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)+(u1*(rho_u*(u1*(k2+k3*rho_u)*2.0+k3*u1*rho_u*2.0)+u1*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u1*(rho_u*(u1*(k5+k6*rho_u)*2.0+k6*u1*rho_u*2.0)+u1*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;

      if (parameterJacobian) {
        // the Jacobian w.r.t. intrinsics parameters
        const double rho_u2 = rho_u*rho_u;
        const double rho_u3 = rho_u*rho_u2;
        Eigen::Matrix2Xd & Jp = *parameterJacobian;
        Jp.resize(2, NumDistortionIntrinsics);
        Jp(0,0) = (u0*rho_u)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);
        Jp(0,1) = (u0*rho_u2)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);
        Jp(0,2) = u0*u1*2.0;
        Jp(0,3) = (u0*u0)*3.0+u1*u1;
        Jp(0,4) = (u0*rho_u3)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);
        Jp(0,5) = -u0*rho_u*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
        Jp(0,6) = -u0*rho_u2*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
        Jp(0,7) = -u0*rho_u3*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
        Jp(1,0) = (u1*rho_u)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);
        Jp(1,1) = (u1*rho_u2)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);
        Jp(1,2) = u0*u0+(u1*u1)*3.0;
        Jp(1,3) = u0*u1*2.0;
        Jp(1,4) = (u1*rho_u3)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);
        Jp(1,5) = -u1*rho_u*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
        Jp(1,6) = -u1*rho_u2*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
        Jp(1,7) = -u1*rho_u3*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;
      }
    return true;
}
bool RadialTangentialDistortion8::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted;  // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  if (!success) {
    std::cout << (E.transpose() * E) << std::endl;
  }

  return success;
}
bool RadialTangentialDistortion8::undistort(
    const Eigen::Vector2d & pointDistorted, Eigen::Vector2d * pointUndistorted,
    Eigen::Matrix2d * pointJacobian) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted;  // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Vector2d dx = (E.transpose() * E).inverse() * E.transpose() * e;

    x_bar += dx;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  // the Jacobian of the inverse map is simply the inverse Jacobian.
  *pointJacobian = E.inverse();

  return success;
}

}  // namespace cameras
}  // namespace okvis
