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
 *  Created on: Aug 21, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#ifndef MOCK_FRONTEND_HPP_
#define MOCK_FRONTEND_HPP_

#define GTEST_USE_OWN_TR1_TUPLE 0
#include "gmock/gmock.h"
#include "gtest/gtest.h"

/// \brief okvis Main namespace of this package.
namespace okvis {

class MockVioFrontendInterface{
 public:
  MOCK_METHOD4(detectAndDescribe,
      bool(size_t cameraIndex, std::shared_ptr<okvis::MultiFrame> frameOut, const okvis::kinematics::Transformation& T_WC, const std::vector<cv::KeyPoint> * keypoints));
  MOCK_METHOD6(dataAssociationAndInitialization,
      bool(okvis::VioBackendInterface& estimator, okvis::kinematics::Transformation& T_WS_propagated, const okvis::VioParameters & params, const std::shared_ptr<okvis::MapPointVector> map, std::shared_ptr<okvis::MultiFrame> framesInOut, bool* asKeyframe));
  MOCK_CONST_METHOD8(propagation,
      bool(const okvis::ImuMeasurementDeque & imuMeasurements, const okvis::ImuParameters & imuParams, okvis::kinematics::Transformation& T_WS_propagated, okvis::SpeedAndBias & speedAndBiases, const okvis::Time& t_start, const okvis::Time& t_end, Eigen::Matrix<double, 15, 15>* covariance, Eigen::Matrix<double, 15, 15>* jacobian));
  MOCK_METHOD1(setBriskDetectionOctaves,
      void(size_t octaves));
  MOCK_METHOD1(setBriskDetectionThreshold,
      void(double threshold));
};

}  // namespace okvis


#endif /* MOCK_DUMMYVIO_HPP_ */
