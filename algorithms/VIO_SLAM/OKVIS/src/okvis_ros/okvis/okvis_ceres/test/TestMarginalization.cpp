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
 *  Created on: Sep 16, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <memory>
#include "glog/logging.h"
#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/MultiFrame.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/NoDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/MarginalizationError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, Marginalization){
  // initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  // set up a random geometry
  std::cout << "set up a random geometry... " << std::flush;
  okvis::kinematics::Transformation T_WS0;  // world to sensor
  T_WS0.setRandom(10.0, M_PI);
  okvis::kinematics::Transformation T_S0S1;
  T_S0S1.setRandom(1.0, 0.01);
  okvis::kinematics::Transformation T_S1S2;
  T_S1S2.setRandom(1.0, 0.01);
  okvis::kinematics::Transformation T_WS1 = T_WS0 * T_S0S1;  // world to sensor
  okvis::kinematics::Transformation T_WS2 = T_WS1 * T_S1S2;  // world to sensor
  okvis::kinematics::Transformation T_disturb;
  T_disturb.setRandom(1, 0.01);
  okvis::kinematics::Transformation T_WS2_init = T_WS2 * T_disturb;  // world to sensor
  okvis::kinematics::Transformation T_SC;  // sensor to camera
  T_SC.setRandom(0.2, M_PI);
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock0_ptr(
      new okvis::ceres::PoseParameterBlock(T_WS0, 0, okvis::Time(0)));
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock1_ptr(
      new okvis::ceres::PoseParameterBlock(T_WS1, 1, okvis::Time(0)));
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock2_ptr(
      new okvis::ceres::PoseParameterBlock(T_WS2_init, 2, okvis::Time(0)));
  std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicsParameterBlock_ptr(
      new okvis::ceres::PoseParameterBlock(T_SC, 3, okvis::Time(0)));

  // use the custom graph/map data structure now:
  okvis::ceres::Map map;
  map.addParameterBlock(poseParameterBlock0_ptr, okvis::ceres::Map::Pose6d);
  map.setParameterBlockConstant(poseParameterBlock0_ptr);
  map.addParameterBlock(poseParameterBlock1_ptr, okvis::ceres::Map::Pose6d);
  map.setParameterBlockConstant(poseParameterBlock1_ptr);
  map.addParameterBlock(poseParameterBlock2_ptr, okvis::ceres::Map::Pose6d);
  map.addParameterBlock(extrinsicsParameterBlock_ptr,
                        okvis::ceres::Map::Pose6d);
  //map.setParameterBlockConstant(extrinsicsParameterBlock_ptr);
  std::cout << " [ OK ] " << std::endl;

  // set up a random camera geometry
  std::cout << "set up a random camera geometry... " << std::flush;
  std::shared_ptr<okvis::cameras::CameraBase> cameraGeometry(
      okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());
  std::cout << " [ OK ] " << std::endl;

  // push the residual blocks to be removed in here:
  std::vector<::ceres::ResidualBlockId> residualBlockIds;

  // add a prior to the extrinsics
  std::shared_ptr<::ceres::CostFunction> extrinsics_prior_cost(
      new okvis::ceres::PoseError(T_SC, 1e-4, 1e-4));
  ::ceres::ResidualBlockId id = map.addResidualBlock(
      extrinsics_prior_cost, NULL, extrinsicsParameterBlock_ptr);
  residualBlockIds.push_back(id);

  // add a prior to the poses
  //std::shared_ptr< ::ceres::CostFunction> pose0_prior_cost(new okvis::ceres::PoseError(T_WS0, 1e-4, 1e-4));
  //id = map.addResidualBlock(pose0_prior_cost, NULL,poseParameterBlock0_ptr);
  //residualBlockIds.push_back(id);
  //std::shared_ptr< ::ceres::CostFunction> pose1_prior_cost(new okvis::ceres::PoseError(T_WS1, 1e-4, 1e-4));
  //id = map.addResidualBlock(pose1_prior_cost, NULL,poseParameterBlock1_ptr);
  //residualBlockIds.push_back(id);

  // get some random points and build error terms
  const size_t N = 100;
  std::vector<uint64_t> marginalizationParametersBlocksIds;
  std::cout << "create N=" << N
            << " visible points and add respective reprojection error terms... "
            << std::flush;
  for (size_t i = 0; i < N; ++i) {
    // points in camera frames:
    Eigen::Vector4d pointC0 = cameraGeometry->createRandomVisibleHomogeneousPoint(10.0);
    Eigen::Vector4d pointC1 = T_SC.inverse() * T_WS1.inverse() * T_WS0 * T_SC
        * pointC0;
    Eigen::Vector4d pointC2 = T_SC.inverse() * T_WS2.inverse() * T_WS0 * T_SC
        * pointC0;

    std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> homogeneousPointParameterBlock_ptr(
        new okvis::ceres::HomogeneousPointParameterBlock(
            T_WS0 * T_SC * pointC0, i + 4));
    map.addParameterBlock(homogeneousPointParameterBlock_ptr,
                          okvis::ceres::Map::HomogeneousPoint);

    // get a randomized projections
    Eigen::Vector2d kp0;
    cameraGeometry->projectHomogeneous(pointC0, &kp0);
    kp0 += Eigen::Vector2d::Random();

    // Set up cost function
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    std::shared_ptr< ::ceres::CostFunction> cost_function0(
        new okvis::ceres::ReprojectionError<okvis::cameras::PinholeCamera<
        okvis::cameras::EquidistantDistortion>>(
            std::static_pointer_cast<
                const okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion>>(cameraGeometry),
            0, kp0, information));

    ::ceres::ResidualBlockId id0 = map.addResidualBlock(
        cost_function0, NULL, poseParameterBlock0_ptr,
        homogeneousPointParameterBlock_ptr, extrinsicsParameterBlock_ptr);

    residualBlockIds.push_back(id0);

    // get a randomized projections
    Eigen::Vector2d kp1;
    if (cameraGeometry->projectHomogeneous(pointC1, &kp1) == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
      kp1 += Eigen::Vector2d::Random();

      // Set up cost function
      Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
      std::shared_ptr< ::ceres::CostFunction> cost_function1(
          new okvis::ceres::ReprojectionError<
          okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion>>(
              std::static_pointer_cast<
                  const okvis::cameras::PinholeCamera<
                      okvis::cameras::EquidistantDistortion>>(cameraGeometry),
              0, kp1, information));

      ::ceres::ResidualBlockId id1 = map.addResidualBlock(
          cost_function1, NULL, poseParameterBlock1_ptr,
          homogeneousPointParameterBlock_ptr, extrinsicsParameterBlock_ptr);

      residualBlockIds.push_back(id1);
    }

    // get a randomized projections
    Eigen::Vector2d kp2;
    if (cameraGeometry->projectHomogeneous(pointC2, &kp2) == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
      kp2 += Eigen::Vector2d::Random();

      // Set up cost function
      Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
      std::shared_ptr< ::ceres::CostFunction> cost_function2(
          new okvis::ceres::ReprojectionError<okvis::cameras::PinholeCamera<
          okvis::cameras::EquidistantDistortion>>(
              std::static_pointer_cast<
                  const okvis::cameras::PinholeCamera<
                      okvis::cameras::EquidistantDistortion>>(cameraGeometry), 0, kp2,
                                                      information));

      ::ceres::ResidualBlockId id2 = map.addResidualBlock(
          cost_function2, NULL, poseParameterBlock2_ptr,
          homogeneousPointParameterBlock_ptr, extrinsicsParameterBlock_ptr);

      residualBlockIds.push_back(id2);
    }

    marginalizationParametersBlocksIds.push_back(
        homogeneousPointParameterBlock_ptr->id());
  }
  std::cout << " [ OK ] " << std::endl;

  // Run the solver!
  std::cout << "run the solver... " << std::endl;
  map.options.minimizer_progress_to_stdout = false;
  ::FLAGS_stderrthreshold = google::WARNING;  // enable console warnings (Jacobian verification)
  map.solve();

  // verify there are no errors in the Jacobians
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

  // print some infos about the optimization
  //std::cout << map.summary.FullReport() << "\n";
  std::cout << "initial T_WS2 : " << T_WS2_init.T() << "\n"
            << "optimized T_WS2 : " << poseParameterBlock2_ptr->estimate().T()
            << "\n" << "correct T_WS : " << T_WS2.T() << "\n";

  // make sure it converged
  OKVIS_ASSERT_TRUE(
      Exception,
      2 * (T_WS2.q() * poseParameterBlock2_ptr->estimate().q().inverse()).vec().norm()
          < 1e-2,
      "quaternions not close enough");
  OKVIS_ASSERT_TRUE(
      Exception,
      (T_WS2.r() - poseParameterBlock2_ptr->estimate().r()).norm() < 1e-1, "translation not close enough");
}

