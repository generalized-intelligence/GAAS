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
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <memory>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ceres/ceres.h>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/HomogeneousPointError.hpp>
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, HomogeneousPointError) {
  // initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  // Build the problem.
  okvis::ceres::Map map;

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

  for (size_t i = 0; i < 100; ++i) {
    // create point
    Eigen::Vector4d point;
    point.head<3>().setRandom();
    point *= 100;
    point[3] = 1.0;

    // create parameter block
    std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> homogeneousPointParameterBlock(
        new okvis::ceres::HomogeneousPointParameterBlock(point, i));
    // add it as optimizable thing.
    map.addParameterBlock(homogeneousPointParameterBlock,
                          okvis::ceres::Map::HomogeneousPoint);
    map.setParameterBlockVariable(i);

    // invent a point error
    std::shared_ptr<okvis::ceres::HomogeneousPointError> homogeneousPointError(
        new okvis::ceres::HomogeneousPointError(
            homogeneousPointParameterBlock->estimate(), 0.1));

    // add it
    ::ceres::ResidualBlockId id = map.addResidualBlock(
        homogeneousPointError, NULL, homogeneousPointParameterBlock);

    // disturb
    Eigen::Vector4d point_disturbed = point;
    point_disturbed.head<3>() += 0.2 * Eigen::Vector3d::Random();
    homogeneousPointParameterBlock->setEstimate(point_disturbed);

    // check Jacobian
    OKVIS_ASSERT_TRUE(Exception, map.isJacobianCorrect(id),
                   "Jacobian verification on homogeneous point error failed.");
  }

  // Run the solver!
  map.options.minimizer_progress_to_stdout = false;
  std::cout << "run the solver... " << std::endl;
  map.solve();

  // print some infos about the optimization
  //std::cout << map.summary.BriefReport() << "\n";

  // check convergence. this must converge to zero, since it is not an overdetermined system.
  OKVIS_ASSERT_TRUE(
      Exception,
      map.summary.final_cost < 1.0e-10,
      "No convergence. this must converge to zero, since it is not an overdetermined system.");
}
