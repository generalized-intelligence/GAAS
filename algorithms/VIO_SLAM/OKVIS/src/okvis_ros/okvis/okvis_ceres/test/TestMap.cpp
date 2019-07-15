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
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <memory>
#include <sys/time.h>
#include "glog/logging.h"
#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, Map)
{
  // initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

  // set up a random geometry
  std::cout << "set up a random geometry... " << std::flush;
  okvis::kinematics::Transformation T_WS;  // world to sensor
  T_WS.setRandom(10.0, M_PI);
  okvis::kinematics::Transformation T_disturb;
  T_disturb.setRandom(1, 0.01);
  okvis::kinematics::Transformation T_WS_init = T_WS * T_disturb;  // world to sensor
  okvis::kinematics::Transformation T_SC;  // sensor to camera
  T_SC.setRandom(0.2, M_PI);
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock_ptr(
      new okvis::ceres::PoseParameterBlock(T_WS_init, 1, okvis::Time(0)));
  std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicsParameterBlock_ptr(
      new okvis::ceres::PoseParameterBlock(T_SC, 2, okvis::Time(0)));

  // use the custom graph/map datastructure now:
  okvis::ceres::Map map;
  map.addParameterBlock(poseParameterBlock_ptr, okvis::ceres::Map::Pose6d);
  map.addParameterBlock(extrinsicsParameterBlock_ptr,
                        okvis::ceres::Map::Pose6d);
  map.setParameterBlockConstant(extrinsicsParameterBlock_ptr);  // do not optimize this...
  std::cout << " [ OK ] " << std::endl;

  // set up a random camera geometry
  std::cout << "set up a random camera geometry... " << std::flush;
  typedef okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion> DistortedPinholeCameraGeometry;
  std::shared_ptr<const DistortedPinholeCameraGeometry> cameraGeometry =
      std::static_pointer_cast<const DistortedPinholeCameraGeometry>(
          DistortedPinholeCameraGeometry::createTestObject());
  std::cout << " [ OK ] " << std::endl;

  // get some random points and build error terms
  const size_t N = 1000;

  std::cout << "create N=" << N
            << " visible points and add respective reprojection error terms... "
            << std::flush;
  ::ceres::CauchyLoss loss(1);
  for (size_t i = 0; i < N; ++i) {

    Eigen::Vector4d point = cameraGeometry->createRandomVisibleHomogeneousPoint(
        double(i % 10) * 3 + 2.0);
    std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> homogeneousPointParameterBlock_ptr(
        new okvis::ceres::HomogeneousPointParameterBlock(
            T_WS * T_SC * point, i + 3));
    map.addParameterBlock(homogeneousPointParameterBlock_ptr,
                          okvis::ceres::Map::HomogeneousPoint);
    map.setParameterBlockConstant(homogeneousPointParameterBlock_ptr);  // no point optimization

    // get a randomized projection
    Eigen::Vector2d kp;
    cameraGeometry->projectHomogeneous(point, &kp);
    kp += Eigen::Vector2d::Random();

    // Set up the only cost function (also known as residual).
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    std::shared_ptr<::ceres::CostFunction> cost_function(
        new okvis::ceres::ReprojectionError<DistortedPinholeCameraGeometry>(
            cameraGeometry, 1, kp, information));

    ::ceres::ResidualBlockId id = map.addResidualBlock(
        cost_function, &loss, poseParameterBlock_ptr,
        homogeneousPointParameterBlock_ptr, extrinsicsParameterBlock_ptr);
    OKVIS_ASSERT_TRUE(Exception,map.isJacobianCorrect(id),"wrong Jacobian");

    if (i % 10 == 0) {
      if (i % 20 == 0)
        map.removeParameterBlock(homogeneousPointParameterBlock_ptr);  // randomly delete some just for fun to test
      else
        map.removeResidualBlock(id);  // randomly delete some just for fun to test
    }
  }
  std::cout << " [ OK ] " << std::endl;

  // Run the solver!
  std::cout << "run the solver... " << std::endl;
  //map.options.check_gradients=true;
  //map.options.numeric_derivative_relative_step_size = 1e-6;
  //map.options.gradient_check_relative_precision=1e-2;
  map.options.minimizer_progress_to_stdout = true;
  map.options.max_num_iterations = 10;
  ::FLAGS_stderrthreshold = google::WARNING;  // enable console warnings (Jacobian verification)
  map.solve();

  // print some infos about the optimization
  //std::cout << map.summary.FullReport() << "\n";
  std::cout << "initial T_WS : " << T_WS_init.T() << "\n" << "optimized T_WS : "
            << poseParameterBlock_ptr->estimate().T() << "\n"
            << "correct T_WS : " << T_WS.T() << "\n";

  // make sure it converged
  OKVIS_ASSERT_TRUE(
      Exception,
      2*(T_WS.q() * poseParameterBlock_ptr->estimate().q().inverse()).vec().norm() < 1e-2,
      "quaternions not close enough");
  OKVIS_ASSERT_TRUE(
      Exception,
      (T_WS.r() - poseParameterBlock_ptr->estimate().r()).norm() < 1e-1,
      "translation not close enough");

  // also try out the resetting of parameterization:
  map.resetParameterization(poseParameterBlock_ptr->id(), okvis::ceres::Map::Pose2d);
  okvis::kinematics::Transformation T_start = poseParameterBlock_ptr->estimate();
  Eigen::Matrix<double, 6, 1> disturb_rp;
  disturb_rp.setZero();
  disturb_rp.segment<2>(3).setRandom();
  disturb_rp.segment<2>(3) *= 0.01;
  T_start.oplus(disturb_rp);
  poseParameterBlock_ptr->setEstimate(T_start);  // disturb again, so it has to optimize
  map.solve();
  //std::cout << map.summary.FullReport() << "\n";

}

