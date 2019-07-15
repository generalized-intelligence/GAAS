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
#include "okvis/MultiFrame.hpp"

TEST(MulitFrame, functions)
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
  okvis::MultiFrame multiFrame(nCameraSystem, okvis::Time::now(), 1);

  for (size_t c = 0; c < cameras.size(); ++c) {
    //std::cout << "Testing MultiFrame with " << cameras.at(c)->type() << std::endl;

#ifdef __ARM_NEON__
   std::shared_ptr<cv::FeatureDetector> detector(
        new brisk::BriskFeatureDetector(34, 2));
#else
   std::shared_ptr<cv::FeatureDetector> detector(
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            34, 2, 800, 450));
#endif

    std::shared_ptr<cv::DescriptorExtractor> extractor(
        new cv::BriskDescriptorExtractor(true, false));

    // create a stupid random image
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> eigenImage(
        752, 480);
    eigenImage.setRandom();
    cv::Mat image(480, 752, CV_8UC1, eigenImage.data());

    // setup multifrmae
    multiFrame.setDetector(c,detector);
    multiFrame.setExtractor(c,extractor);
    multiFrame.setImage(c,image);

    // run
    multiFrame.detect(c);
    multiFrame.describe(c);
  }
}

