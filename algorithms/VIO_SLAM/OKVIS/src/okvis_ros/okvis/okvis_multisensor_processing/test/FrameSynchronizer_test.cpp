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
 *  Created on: Sep 14, 2014
 *      Author: Pascal Gohl
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include "gtest/gtest.h"
#include <memory>
#include "testDataGenerators.hpp"
#include "okvis/FrameSynchronizer.hpp"
#include <glog/logging.h>

using namespace okvis;

static const size_t num_cameras = 2;
static const size_t num_test_frames = 10;

class FrameSynchronizerTest : public ::testing::Test {
 protected:

  FrameSynchronizerTest()
 : parameters(),
   frame_syncer(parameters){
    parameters.nCameraSystem = TestDataGenerator::getTestCameraSystem(num_cameras);
    frame_syncer.init(parameters);
  }

  virtual ~FrameSynchronizerTest() {
  }

  virtual void SetUp() {
    double now = 0.1;//okvis::Time::now().toSec();

    // test Observations
    cv::Mat image_cam;
    image_cam = cv::imread("testImage.jpg", 0);
    if (image_cam.data == NULL) {
      std::cout << "testImage.jpg not found" << std::endl;
    }

    ASSERT_TRUE(image_cam.data != NULL);

    test_frames.resize(num_test_frames);
    for (size_t i = 0; i < num_test_frames; ++i) {
      for (size_t j = 0; j < num_cameras; ++j) {
        std::shared_ptr<okvis::CameraMeasurement> new_frame = std::make_shared<okvis::CameraMeasurement>();
        new_frame->sensorId = j;
        new_frame->timeStamp = okvis::Time(now);
        new_frame->measurement.image = image_cam.clone();
        new_frame->measurement.image.data[0] = i; // store frame data in image
        new_frame->measurement.image.data[1] = j;

        test_frames[i].push_back(new_frame);
      }

      now += 1;
    }
  }

  virtual void TearDown() {
  }

  std::vector<std::vector<std::shared_ptr<okvis::CameraMeasurement> > > test_frames;
  okvis::VioParameters parameters;
  FrameSynchronizer frame_syncer;

};

TEST_F(FrameSynchronizerTest, ConstructDestruct)
{
}

TEST_F(FrameSynchronizerTest, CorrectOrder)
{
  for (size_t i = 0; i < num_test_frames; ++i) {
    okvis::MultiFramePtr multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(0));
    frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
    EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(1));
    frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
    EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));

    EXPECT_EQ(i, multiFrame->image(0).data[0]);
    EXPECT_EQ(0, multiFrame->image(0).data[1]);
    EXPECT_EQ(i, multiFrame->image(1).data[0]);
    EXPECT_EQ(1, multiFrame->image(1).data[1]);
  }
}

TEST_F(FrameSynchronizerTest, OneMissing)
{
  for (size_t i = 0; i < num_test_frames; ++i) {
    okvis::MultiFramePtr multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(0));
    frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
    EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    if(i == 3) {
      EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    } else {
      multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(1));
      frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
      EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));

      EXPECT_EQ(i, multiFrame->image(0).data[0]);
      EXPECT_EQ(0, multiFrame->image(0).data[1]);
      EXPECT_EQ(i, multiFrame->image(1).data[0]);
      EXPECT_EQ(1, multiFrame->image(1).data[1]);
    }
  }
}

TEST_F(FrameSynchronizerTest, TwoMissing)
{
  for (size_t i = 0; i < num_test_frames; ++i) {
    okvis::MultiFramePtr multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(0));
    frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
    EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    if(i == 3 || i == 5) {
      EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    } else {
      multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(1));
      frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
      EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));

      EXPECT_EQ(i, multiFrame->image(0).data[0]);
      EXPECT_EQ(0, multiFrame->image(0).data[1]);
      EXPECT_EQ(i, multiFrame->image(1).data[0]);
      EXPECT_EQ(1, multiFrame->image(1).data[1]);
    }
  }
}

TEST_F(FrameSynchronizerTest, OneDouble)
{
  for (size_t i = 0; i < num_test_frames; ++i) {
    okvis::MultiFramePtr multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(0));
    frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
    EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    if(i == 3) {
      multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(1));
      frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
      multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(1));
      frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
      // expecting false as something unexpected happened
      // (3 frames with the same timestamp detected in a 2 camera setup)
      // and frame synchronizer wants you to drop the multiframe
      EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    } else {
      multiFrame = frame_syncer.addNewFrame(test_frames.at(i).at(1));
      frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
      EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));
    }

    EXPECT_EQ(i, multiFrame->image(0).data[0]);
    EXPECT_EQ(0, multiFrame->image(0).data[1]);
    EXPECT_EQ(i, multiFrame->image(1).data[0]);
    EXPECT_EQ(1, multiFrame->image(1).data[1]);
  }
}

TEST_F(FrameSynchronizerTest, OneOutOfOrder)
{
  okvis::MultiFramePtr multiFrame;
  multiFrame = frame_syncer.addNewFrame(test_frames.at(0).at(0));
  frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
  multiFrame = frame_syncer.addNewFrame(test_frames.at(0).at(1));
  frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
  EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));

  multiFrame = frame_syncer.addNewFrame(test_frames.at(1).at(0));
  frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
  multiFrame = frame_syncer.addNewFrame(test_frames.at(1).at(1));
  frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
  EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));

  multiFrame = frame_syncer.addNewFrame(test_frames.at(2).at(0));
  frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
  multiFrame = frame_syncer.addNewFrame(test_frames.at(2).at(1));
  frame_syncer.detectionEndedForMultiFrame(multiFrame->id());
  EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame->id()));

  okvis::MultiFramePtr multiFrame3 = frame_syncer.addNewFrame(test_frames.at(3).at(0));
  frame_syncer.detectionEndedForMultiFrame(multiFrame3->id());
  okvis::MultiFramePtr multiFrame4 = frame_syncer.addNewFrame(test_frames.at(4).at(1));
  frame_syncer.detectionEndedForMultiFrame(multiFrame4->id());
  EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame3->id()));
  EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame4->id()));

  multiFrame4 = frame_syncer.addNewFrame(test_frames.at(4).at(0));
  frame_syncer.detectionEndedForMultiFrame(multiFrame4->id());
  EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame3->id()));
  EXPECT_TRUE(frame_syncer.detectionCompletedForAllCameras(multiFrame4->id()));
  EXPECT_EQ(4, multiFrame4->image(0).data[0]);
  EXPECT_EQ(0, multiFrame4->image(0).data[1]);
  EXPECT_EQ(4, multiFrame4->image(1).data[0]);
  EXPECT_EQ(1, multiFrame4->image(1).data[1]);

  multiFrame3 = frame_syncer.addNewFrame(test_frames.at(3).at(1));
  frame_syncer.detectionEndedForMultiFrame(multiFrame3->id());
  // This will result in an assertion failing as the order is wrong.
//  EXPECT_FALSE(frame_syncer.detectionCompletedForAllCameras(multiFrame3->id()));

  frame_syncer.addNewFrame(test_frames.at(5).at(0));
  frame_syncer.addNewFrame(test_frames.at(5).at(1));

  frame_syncer.addNewFrame(test_frames.at(6).at(0));
  frame_syncer.addNewFrame(test_frames.at(6).at(1));
}

