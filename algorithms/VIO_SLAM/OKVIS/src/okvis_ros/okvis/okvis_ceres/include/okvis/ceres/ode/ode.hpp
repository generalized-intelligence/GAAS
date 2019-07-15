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
 *  Created on: Jan 7, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file ode.hpp
 * @brief File for ODE integration functionality.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_ODE_ODE_HPP_
#define INCLUDE_OKVIS_CERES_ODE_ODE_HPP_

#include <Eigen/Core>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Variables.hpp>
#include <okvis/assert_macros.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {
/// \brief ode Namespace for functionality related to ODE integration implemented in okvis.
namespace ode {

// to make things a bit faster than using angle-axis conversion:
__inline__ double sinc(double x) {
  if (fabs(x) > 1e-6) {
   return sin(x) / x;
   } else{
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

// world-centric velocities
__inline__ void evaluateContinuousTimeOde(const Eigen::Vector3d& gyr, const Eigen::Vector3d& acc, double g,
                                          const Eigen::Vector3d& p_WS_W, const Eigen::Quaterniond& q_WS,
                                          const okvis::SpeedAndBias& sb, Eigen::Vector3d& p_WS_W_dot,
                                          Eigen::Vector4d& q_WS_dot, okvis::SpeedAndBias& sb_dot,
                                          Eigen::Matrix<double, 15, 15>* F_c_ptr = 0){
  // "true" rates and accelerations
  const Eigen::Vector3d omega_S = gyr - sb.segment<3>(3);
  const Eigen::Vector3d acc_S = acc - sb.tail<3>();

  // nonlinear states
  // start with the pose
  p_WS_W_dot = sb.head<3>();

  // now the quaternion
  Eigen::Vector4d dq;
  q_WS_dot.head<3>() = 0.5 * omega_S;
  q_WS_dot[3] = 0.0;
  Eigen::Matrix3d C_WS = q_WS.toRotationMatrix();

  // the rest is straightforward
  // consider Earth's radius. Model the Earth as a sphere, since we neither
  // know the position nor yaw (except if coupled with GPS and magnetometer).
  Eigen::Vector3d G = -p_WS_W - Eigen::Vector3d(0, 0, 6371009); // vector to Earth center
  sb_dot.head<3>() = (C_WS * acc_S + g * G.normalized()); // s
  // biases
  sb_dot.tail<6>().setZero();

  // linearized system:
  if (F_c_ptr) {
    F_c_ptr->setZero();
    F_c_ptr->block<3, 3>(0, 6) += Eigen::Matrix3d::Identity();
    F_c_ptr->block<3, 3>(3, 9) -= C_WS;
    F_c_ptr->block<3, 3>(6, 3) -= okvis::kinematics::crossMx(C_WS * acc_S);
    F_c_ptr->block<3, 3>(6, 12) -= C_WS;
  }
}


/*/ robo-centric velocities
__inline__ void evaluateContinuousTimeOde(
    const Eigen::Vector3d& gyr, const Eigen::Vector3d& acc, double g,
    const Eigen::Vector3d& p_WS_W, const Eigen::Quaterniond& q_WS,
    const okvis::SpeedAndBias& sb, Eigen::Vector3d& p_WS_W_dot,
    Eigen::Vector4d& q_WS_dot, okvis::SpeedAndBias& sb_dot,
    Eigen::Matrix<double, 15, 15>* F_c_ptr = 0) {

  // "true" rates and accelerations
  const Eigen::Vector3d omega_S = gyr - sb.segment<3>(3);
  const Eigen::Vector3d acc_S = acc - sb.tail<3>();

  // rotation matrix
  Eigen::Matrix3d C_WS = q_WS.toRotationMatrix();
  Eigen::Matrix3d C_SW = C_WS.transpose();

  // nonlinear states
  // start with the pose
  p_WS_W_dot = C_WS*sb.head<3>();

  // now the quaternion
  Eigen::Vector4d dq;
  q_WS_dot.head<3>() = 0.5 * omega_S;
  q_WS_dot[3] = 0.0;

  // the rest is straightforward
  // consider Earth's radius. Model the Earth as a sphere, since we neither
  // know the position nor yaw (except if coupled with GPS and magnetometer).
  Eigen::Vector3d G = -p_WS_W - Eigen::Vector3d(0, 0, 6371009);  // vector to Earth center
  Eigen::Vector3d g_W = g * G.normalized();
  sb_dot.head<3>() = acc_S - okvis::kinematics::crossMx(omega_S)*sb.head<3>() + C_SW * g_W;
  // biases
  sb_dot.tail<6>().setZero();
  //sb_dot.tail<3>()=-sb.tail<3>()/360.0;

  // linearized system:
  if (F_c_ptr) {
    F_c_ptr->setZero();
    F_c_ptr->block<3, 3>(0, 3)  += -okvis::kinematics::crossMx(C_WS * sb.head<3>());
    F_c_ptr->block<3, 3>(0, 6)  += C_WS;
    F_c_ptr->block<3, 3>(3, 9)  += -C_WS;
    F_c_ptr->block<3, 3>(6, 3)  += C_SW * okvis::kinematics::crossMx(g_W);
    F_c_ptr->block<3, 3>(6, 6)  += -okvis::kinematics::crossMx(omega_S);
    F_c_ptr->block<3, 3>(6, 9)  += -okvis::kinematics::crossMx(sb.head<3>());
    F_c_ptr->block<3, 3>(6, 12) += -Eigen::Matrix3d::Identity();
    //F_c_ptr->block<3, 3>(12, 12) = - 1.0/360.0*Eigen::Matrix3d::Identity();
  }
}*/

__inline__ void integrateOneStep_RungeKutta(
    const Eigen::Vector3d& gyr_0, const Eigen::Vector3d& acc_0,
    const Eigen::Vector3d& gyr_1, const Eigen::Vector3d& acc_1, double g,
    double sigma_g_c, double sigma_a_c, double sigma_gw_c, double sigma_aw_c,
    double dt, Eigen::Vector3d& p_WS_W, Eigen::Quaterniond& q_WS,
    okvis::SpeedAndBias& sb, Eigen::Matrix<double, 15, 15>* P_ptr = 0,
    Eigen::Matrix<double, 15, 15>* F_tot_ptr = 0) {

  Eigen::Vector3d k1_p_WS_W_dot;
  Eigen::Vector4d k1_q_WS_dot;
  okvis::SpeedAndBias k1_sb_dot;
  Eigen::Matrix<double, 15, 15> k1_F_c;
  evaluateContinuousTimeOde(gyr_0, acc_0, g, p_WS_W, q_WS, sb, k1_p_WS_W_dot,
                            k1_q_WS_dot, k1_sb_dot, &k1_F_c);

  Eigen::Vector3d p_WS_W1 = p_WS_W;
  Eigen::Quaterniond q_WS1 = q_WS;
  okvis::SpeedAndBias sb1 = sb;
  // state propagation:
  p_WS_W1 += k1_p_WS_W_dot * 0.5 * dt;
  Eigen::Quaterniond dq;
  double theta_half = k1_q_WS_dot.head<3>().norm() * 0.5 * dt;
  double sinc_theta_half = sinc(theta_half);
  double cos_theta_half = cos(theta_half);
  dq.vec() = sinc_theta_half * k1_q_WS_dot.head<3>() * 0.5 * dt;
  dq.w() = cos_theta_half;
  q_WS1 = q_WS * dq;
  sb1 += k1_sb_dot * 0.5 * dt;

  Eigen::Vector3d k2_p_WS_W_dot;
  Eigen::Vector4d k2_q_WS_dot;
  okvis::SpeedAndBias k2_sb_dot;
  Eigen::Matrix<double, 15, 15> k2_F_c;
  evaluateContinuousTimeOde(0.5 * (gyr_0 + gyr_1), 0.5 * (acc_0 + acc_1), g,
                            p_WS_W1, q_WS1, sb1, k2_p_WS_W_dot, k2_q_WS_dot,
                            k2_sb_dot, &k2_F_c);

  Eigen::Vector3d p_WS_W2 = p_WS_W;
  Eigen::Quaterniond q_WS2 = q_WS;
  okvis::SpeedAndBias sb2 = sb;
  // state propagation:
  p_WS_W2 += k2_p_WS_W_dot * dt;
  theta_half = k2_q_WS_dot.head<3>().norm() * dt;
  sinc_theta_half = sinc(theta_half);
  cos_theta_half = cos(theta_half);
  dq.vec() = sinc_theta_half * k2_q_WS_dot.head<3>() * dt;
  dq.w() = cos_theta_half;
  //std::cout<<dq.transpose()<<std::endl;
  q_WS2 = q_WS2 * dq;
  sb2 += k1_sb_dot * dt;

  Eigen::Vector3d k3_p_WS_W_dot;
  Eigen::Vector4d k3_q_WS_dot;
  okvis::SpeedAndBias k3_sb_dot;
  Eigen::Matrix<double, 15, 15> k3_F_c;
  evaluateContinuousTimeOde(0.5 * (gyr_0 + gyr_1), 0.5 * (acc_0 + acc_1), g,
                            p_WS_W2, q_WS2, sb2, k3_p_WS_W_dot, k3_q_WS_dot,
                            k3_sb_dot, &k3_F_c);

  Eigen::Vector3d p_WS_W3 = p_WS_W;
  Eigen::Quaterniond q_WS3 = q_WS;
  okvis::SpeedAndBias sb3 = sb;
  // state propagation:
  p_WS_W3 += k3_p_WS_W_dot * dt;
  theta_half = k3_q_WS_dot.head<3>().norm() * dt;
  sinc_theta_half = sinc(theta_half);
  cos_theta_half = cos(theta_half);
  dq.vec() = sinc_theta_half * k3_q_WS_dot.head<3>() * dt;
  dq.w() = cos_theta_half;
  //std::cout<<dq.transpose()<<std::endl;
  q_WS3 = q_WS3 * dq;
  sb3 += k3_sb_dot * dt;

  Eigen::Vector3d k4_p_WS_W_dot;
  Eigen::Vector4d k4_q_WS_dot;
  okvis::SpeedAndBias k4_sb_dot;
  Eigen::Matrix<double, 15, 15> k4_F_c;
  evaluateContinuousTimeOde(gyr_1, acc_1, g, p_WS_W3, q_WS3, sb3, k4_p_WS_W_dot,
                            k4_q_WS_dot, k4_sb_dot, &k4_F_c);

  // now assemble
  p_WS_W +=
      (k1_p_WS_W_dot + 2 * (k2_p_WS_W_dot + k3_p_WS_W_dot) + k4_p_WS_W_dot) * dt
          / 6.0;
  Eigen::Vector3d theta_half_vec = (k1_q_WS_dot.head<3>()
      + 2 * (k2_q_WS_dot.head<3>() + k3_q_WS_dot.head<3>())
      + k4_q_WS_dot.head<3>()) * dt / 6.0;
  theta_half = theta_half_vec.norm();
  sinc_theta_half = sinc(theta_half);
  cos_theta_half = cos(theta_half);
  dq.vec() = sinc_theta_half * theta_half_vec;
  dq.w() = cos_theta_half;
  q_WS = q_WS * dq;
  sb += (k1_sb_dot + 2 * (k2_sb_dot + k3_sb_dot) + k4_sb_dot) * dt / 6.0;

  q_WS.normalize(); // do not accumulate errors!

  if (F_tot_ptr) {
    // compute state transition matrix
    Eigen::Matrix<double, 15, 15>& F_tot = *F_tot_ptr;
    const Eigen::Matrix<double, 15, 15>& J1 = k1_F_c;
    const Eigen::Matrix<double, 15, 15> J2=k2_F_c*(Eigen::Matrix<double, 15, 15>::Identity()+0.5*dt*J1);
    const Eigen::Matrix<double, 15, 15> J3=k3_F_c*(Eigen::Matrix<double, 15, 15>::Identity()+0.5*dt*J2);
    const Eigen::Matrix<double, 15, 15> J4=k4_F_c*(Eigen::Matrix<double, 15, 15>::Identity()+dt*J3);
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity()
        + dt * (J1+2*(J2+J3)+J4) / 6.0;
        //+ dt * J1;
    //std::cout<<F<<std::endl;
    F_tot = (F * F_tot).eval();

    if (P_ptr) {
      Eigen::Matrix<double, 15, 15>& cov = *P_ptr;
      cov = F * (cov * F.transpose()).eval();

      // add process noise
      const double Q_g = sigma_g_c * sigma_g_c * dt;
      const double Q_a = sigma_a_c * sigma_a_c * dt;
      const double Q_gw = sigma_gw_c * sigma_gw_c * dt;
      const double Q_aw = sigma_aw_c * sigma_aw_c * dt;
      cov(3, 3) += Q_g;
      cov(4, 4) += Q_g;
      cov(5, 5) += Q_g;
      cov(6, 6) += Q_a;
      cov(7, 7) += Q_a;
      cov(8, 8) += Q_a;
      cov(9, 9) += Q_gw;
      cov(10, 10) += Q_gw;
      cov(11, 11) += Q_gw;
      cov(12, 12) += Q_aw;
      cov(13, 13) += Q_aw;
      cov(14, 14) += Q_aw;

      // force symmetric - TODO: is this really needed here?
      // cov = 0.5 * cov + 0.5 * cov.transpose().eval();
    }
  }

}

}

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_ODE_ODE_HPP_ */
