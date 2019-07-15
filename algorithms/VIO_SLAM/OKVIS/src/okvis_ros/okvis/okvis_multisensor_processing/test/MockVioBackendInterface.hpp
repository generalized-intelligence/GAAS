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

#ifndef MockVioBackendInterface_HPP_
#define MockVioBackendInterface_HPP_

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <okvis/VioBackendInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

class MockVioBackendInterface : public VioBackendInterface {
 public:
  MOCK_METHOD1(addCamera,
      int(const ExtrinsicsEstimationParameters & extrinsicsEstimationParameters));
  MOCK_METHOD1(addImu,
      int(const ImuParameters & imuParameters));
  MOCK_METHOD0(clearCameras,
      void());
  MOCK_METHOD0(clearImus,
      void());

  MOCK_METHOD3(addStates,
               bool(okvis::MultiFramePtr multiFrame, const okvis::ImuMeasurementDeque & imuMeasurements, bool asKeyframe));

  MOCK_METHOD2(addLandmark,
      bool(uint64_t landmarkId, const Eigen::Vector4d & landmark));
  MOCK_METHOD4(addObservation,
      ::ceres::ResidualBlockId(uint64_t landmarkId, uint64_t poseId, size_t camIdx, size_t keypointIdx));
  MOCK_METHOD4(removeObservation,
      bool(uint64_t landmarkId, uint64_t poseId, size_t camIdx, size_t keypointIdx));
  MOCK_METHOD3(applyMarginalizationStrategy,
      bool(size_t numKeyframes, size_t numImuFrames, okvis::MapPointVector& removedLandmarks));
  MOCK_METHOD3(optimize,
      void(size_t, size_t, bool));
  MOCK_METHOD2(setOptimizationTimeLimit,
      bool(double timeLimit, int minIterations));
  MOCK_CONST_METHOD1(isLandmarkAdded,
      bool(uint64_t landmarkId));
  MOCK_CONST_METHOD1(isLandmarkInitialized,
      bool(uint64_t landmarkId));
  MOCK_CONST_METHOD2(getLandmark,
      bool(uint64_t landmarkId, MapPoint& mapPoint));
  MOCK_CONST_METHOD1(getLandmarks,
      size_t(PointMap & landmarks));
  MOCK_CONST_METHOD1(getLandmarks,
      size_t(okvis::MapPointVector& landmarks));
  MOCK_CONST_METHOD1(multiFrame,
                     okvis::MultiFramePtr(uint64_t frameId));
  MOCK_CONST_METHOD2(get_T_WS,
      bool(uint64_t poseId, okvis::kinematics::Transformation & T_WS));
  MOCK_CONST_METHOD3(getSpeedAndBias,
      bool(uint64_t poseId, uint64_t imuIdx, okvis::SpeedAndBias & speedAndBias));
  MOCK_CONST_METHOD3(getCameraSensorStates,
      bool(uint64_t poseId, size_t cameraIdx, okvis::kinematics::Transformation & T_SCi));
  MOCK_CONST_METHOD0(numFrames,
                     size_t());
  MOCK_CONST_METHOD0(numLandmarks,
      size_t());
  MOCK_CONST_METHOD0(currentKeyframeId,
      uint64_t());
  MOCK_CONST_METHOD1(frameIdByAge,
      uint64_t(size_t age));
  MOCK_CONST_METHOD0(currentFrameId,
      uint64_t());
  MOCK_CONST_METHOD1(isKeyframe,
      bool(uint64_t frameId));
  MOCK_CONST_METHOD1(isInImuWindow,
      bool(uint64_t frameId));
  MOCK_CONST_METHOD1(timestamp,
      okvis::Time(uint64_t frameId));
  MOCK_METHOD2(set_T_WS,
      bool(uint64_t poseId, const okvis::kinematics::Transformation & T_WS));
  MOCK_METHOD3(setSpeedAndBias,
      bool(uint64_t poseId, size_t imuIdx, const okvis::SpeedAndBias & speedAndBias));
  MOCK_METHOD3(setCameraSensorStates,
      bool(uint64_t poseId, size_t cameraIdx, const okvis::kinematics::Transformation & T_SCi));
  MOCK_METHOD2(setLandmark,
      bool(uint64_t landmarkId, const Eigen::Vector4d & landmark));
  MOCK_METHOD2(setLandmarkInitialized,
      void(uint64_t landmarkId, bool initialized));
  MOCK_METHOD2(setKeyframe,
      void(uint64_t frameId, bool isKeyframe));
  MOCK_METHOD1(setMap,
      void(std::shared_ptr<okvis::ceres::Map> mapPtr));
  MOCK_CONST_METHOD0(initializationStatus, 
      VioBackendInterface::InitializationStatus());
};

}  // namespace okvis

#endif /* MockVioBackendInterface_HPP_ */
