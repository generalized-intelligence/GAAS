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
 *  Created on: Dec 5, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <okvis/kinematics/Transformation.hpp>
#include <iostream>
#include "gtest/gtest.h"

TEST(Transformation, operations) {
  for (size_t i = 0; i < 100; ++i) {
    okvis::kinematics::Transformation T_AB;
    T_AB.setRandom();
    okvis::kinematics::Transformation T_BC;
    T_BC.setRandom();

    // Test inverse
    EXPECT_TRUE(
        ((T_AB * T_AB.inverse()).T() - Eigen::Matrix4d::Identity()).norm()
            < 1e-8);

    // Test composition
    EXPECT_TRUE(((T_AB * T_BC).T() - T_AB.T() * T_BC.T()).norm() < 1e-8);

    // Test construction
    okvis::kinematics::Transformation T_AB_alternative(T_AB.T());
    EXPECT_TRUE((T_AB.T() - T_AB_alternative.T()).norm() < 1e-8);
    okvis::kinematics::Transformation T_AB_alternative2(T_AB.r(), T_AB.q());
    EXPECT_TRUE((T_AB.T() - T_AB_alternative2.T()).norm() < 1e-8);

    // Test =
    okvis::kinematics::Transformation T_AB_alternative3;
    T_AB_alternative3 = T_AB;
    EXPECT_TRUE((T_AB.T() - T_AB_alternative3.T()).norm() < 1e-8);

    // Test setters
    okvis::kinematics::Transformation T_AB_alternative4;
    T_AB_alternative4.set(T_AB.r(), T_AB.q());
    EXPECT_TRUE((T_AB.T() - T_AB_alternative4.T()).norm() < 1e-8);
    okvis::kinematics::Transformation T_AB_alternative5;
    T_AB_alternative5.set(T_AB.T());
    EXPECT_TRUE((T_AB.T() - T_AB_alternative5.T()).norm() < 1e-8);

    T_AB.setRandom();

    // Test oplus
    const double dp = 1.0e-6;
    Eigen::Matrix<double, 7, 6, Eigen::RowMajor> jacobian_numDiff;
    for (size_t i = 0; i < 6; ++i) {
      okvis::kinematics::Transformation T_AB_p = T_AB;
      okvis::kinematics::Transformation T_AB_m = T_AB;
      Eigen::Matrix<double, 6, 1> dp_p;
      Eigen::Matrix<double, 6, 1> dp_m;
      dp_p.setZero();
      dp_m.setZero();
      dp_p[i] = dp;
      dp_m[i] = -dp;
      T_AB_p.oplus(dp_p);
      T_AB_m.oplus(dp_m);
      /*jacobian_numDiff.block<7, 1>(0, i) = (T_AB_p.parameters()
          - T_AB_m.parameters()) / (2.0 * dp);*/
      jacobian_numDiff.block<3, 1>(0, i) = (T_AB_p.r() - T_AB_m.r())
          / (2.0 * dp);
      jacobian_numDiff.block<4, 1>(3, i) = (T_AB_p.q().coeffs()
          - T_AB_m.q().coeffs()) / (2.0 * dp);
    }
    Eigen::Matrix<double, 7, 6, Eigen::RowMajor> jacobian;
    T_AB.oplusJacobian(jacobian);
    //std::cout << jacobian << std::endl;
    //std::cout << jacobian_numDiff << std::endl;
    EXPECT_TRUE((jacobian - jacobian_numDiff).norm() < 1e-8);
    // also check lift Jacobian: dChi/dx*dx/dChi == 1
    Eigen::Matrix<double, 6, 7, Eigen::RowMajor> lift_jacobian;
    T_AB.liftJacobian(lift_jacobian);
    EXPECT_TRUE(
        (lift_jacobian * jacobian - Eigen::Matrix<double, 6, 6>::Identity())
            .norm() < 1e-8);

    // Test minus
    okvis::kinematics::Transformation T_AB_disturbed = T_AB;
    Eigen::Matrix<double, 6, 1> delta, delta2;
    delta.setRandom();
    delta *= 0.1;  // quite large disturbance
    T_AB_disturbed.oplus(delta);
    // get numeric Jacobian
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobianMinus_numDiff;
    /*for(size_t i=0; i<6; ++i){
     okvis::kinematics::Transformation T_AB_p = T_AB_disturbed;
     okvis::kinematics::Transformation T_AB_m = T_AB_disturbed;
     Eigen::Matrix<double,6,1> dp_p;
     Eigen::Matrix<double,6,1> dp_m;
     dp_p.setZero();
     dp_m.setZero();
     dp_p[i] = dp;
     dp_m[i] = -dp;
     T_AB_p.oplus(dp_p);
     T_AB_m.oplus(dp_m);
     }*/
    //Eigen::Matrix<double, 6, 6, Eigen::RowMajor> minusJacobian;
    //T_AB_disturbed.minus(T_AB, delta2, minusJacobian);
    //EXPECT_TRUE((delta - delta2).norm() < 1e-8);
  }
}
