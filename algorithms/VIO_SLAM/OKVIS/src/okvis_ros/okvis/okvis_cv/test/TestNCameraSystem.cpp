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
 *  Created on: Mar 31, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <brisk/brisk.h>
#pragma GCC diagnostic pop
#include "okvis/cameras/PinholeCamera.hpp"
#include "okvis/cameras/NoDistortion.hpp"
#include "okvis/cameras/RadialTangentialDistortion.hpp"
#include "okvis/cameras/EquidistantDistortion.hpp"
#include "okvis/cameras/NCameraSystem.hpp"

TEST(NCameraSystem, functions)
{

  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<const okvis::cameras::CameraBase> > cameras;
  std::vector<okvis::cameras::NCameraSystem::DistortionType> distortions;
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::NoDistortion>::createTestObject());
  distortions.push_back(okvis::cameras::NCameraSystem::NoDistortion);
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>::createTestObject());
  distortions.push_back(okvis::cameras::NCameraSystem::RadialTangential);
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());
  distortions.push_back(okvis::cameras::NCameraSystem::Equidistant);

  // the mounting transformations. The third one is opposite direction
  std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> T_SC;
  T_SC.push_back(
      std::shared_ptr<okvis::kinematics::Transformation>(
          new okvis::kinematics::Transformation(Eigen::Vector3d(0.1, 0.1, 0.1),
                                                Eigen::Quaterniond(1, 0, 0, 0))));
  T_SC.push_back(
      std::shared_ptr<okvis::kinematics::Transformation>(
          new okvis::kinematics::Transformation(
              Eigen::Vector3d(0.1, -0.1, -0.1), Eigen::Quaterniond(1, 0, 0, 0))));
  T_SC.push_back(
      std::shared_ptr<okvis::kinematics::Transformation>(
          new okvis::kinematics::Transformation(
              Eigen::Vector3d(0.1, -0.1, -0.1), Eigen::Quaterniond(0, 0, 1, 0))));

  okvis::cameras::NCameraSystem nCameraSystem(T_SC, cameras, distortions, true);  // computes overlaps

  // verify self overlaps
  OKVIS_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(0, 0),
                    "No self overlap?");
  OKVIS_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(1, 1),
                    "No self overlap?");
  OKVIS_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(2, 2),
                    "No self overlap?");

  // verify 0 and 1 overlap
  OKVIS_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(0, 1),
                    "No overlap?");
  OKVIS_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(1, 0),
                    "No overlap?");

  // verify 1 and 2 do not overlap
  OKVIS_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(1, 2),
                    "Overlap?");
  OKVIS_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(2, 1),
                    "Overlap?");

  // verify 0 and 2 do not overlap
  OKVIS_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(0, 2),
                    "Overlap?");
  OKVIS_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(2, 0),
                    "Overlap?");

}

