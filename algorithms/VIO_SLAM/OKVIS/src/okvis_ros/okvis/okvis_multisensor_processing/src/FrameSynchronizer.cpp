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
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file FrameSynchronizer.cpp
 * @brief Source file for the FrameSynchronizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>

#include <okvis/FrameSynchronizer.hpp>
#include <okvis/IdProvider.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

static const int max_frame_sync_buffer_size = 3;

// Constructor. Calls init().
FrameSynchronizer::FrameSynchronizer(okvis::VioParameters& parameters)
: parameters_(parameters),
  numCameras_(0),
  timeTol_(parameters.sensors_information.frameTimestampTolerance),
  lastCompletedFrameId_(0)
{
  if(parameters.nCameraSystem.numCameras() > 0) {
    init(parameters);
  }
  frameBuffer_.resize(max_frame_sync_buffer_size,
                      std::pair<std::shared_ptr<okvis::MultiFrame>,size_t>(nullptr,0));
  bufferPosition_ = 0;
}

// Trivial destructor.
FrameSynchronizer::~FrameSynchronizer() {
}

// Initialise the synchronizer with new parameters. Is called in the constructor.
void FrameSynchronizer::init(okvis::VioParameters& parameters) {
  parameters_ = parameters;
  numCameras_ = parameters.nCameraSystem.numCameras();
  timeTol_ = parameters.sensors_information.frameTimestampTolerance;
  // TODO(gohlp): this fails if camera id's are not consecutive
}

// Adds a new frame to the internal buffer and returns the Multiframe containing the frame.
std::shared_ptr<okvis::MultiFrame> FrameSynchronizer::addNewFrame(std::shared_ptr<okvis::CameraMeasurement>& frame) {
  assert(numCameras_ > 0);
  okvis::Time frame_stamp = frame->timeStamp;
  std::shared_ptr<okvis::MultiFrame> multiFrame;
  int position;
  if(findFrameByTime(frame_stamp,position)) {
    multiFrame = frameBuffer_[position].first;
    OKVIS_ASSERT_TRUE_DBG(Exception,multiFrame->image(frame->sensorId).empty(),
                       "Frame for this camera has already been added to multiframe!");
    if(frame_stamp != multiFrame->timestamp()) {
      // timestamps do not agree. setting timestamp to middlepoint
      frame_stamp += (multiFrame->timestamp()-frame_stamp)*0.5;
      multiFrame->setTimestamp(frame_stamp);
    }
    multiFrame->setImage(frame->sensorId,frame->measurement.image);
  }
  else {
    multiFrame = std::shared_ptr<okvis::MultiFrame>(new okvis::MultiFrame(parameters_.nCameraSystem,frame_stamp,
                                                                          okvis::IdProvider::instance().newId()));
    multiFrame->setImage(frame->sensorId,frame->measurement.image);
    bufferPosition_ = (bufferPosition_+1) % max_frame_sync_buffer_size;
    if(frameBuffer_[bufferPosition_].first != nullptr
       && frameBuffer_[bufferPosition_].second != numCameras_) {
     LOG(ERROR) << "Dropping frame with id " << frameBuffer_[bufferPosition_].first->id();
    }
    frameBuffer_[bufferPosition_].first = multiFrame;
    frameBuffer_[bufferPosition_].second= 0;
  }
  return multiFrame;
}

// Inform the synchronizer that a frame in the multiframe has completed keypoint detection and description.
bool FrameSynchronizer::detectionEndedForMultiFrame(uint64_t multiFrameId) {
  int position;
  bool found = findFrameById(multiFrameId,position);
  if(found) {
    ++frameBuffer_[position].second;
    OKVIS_ASSERT_TRUE_DBG(Exception,frameBuffer_[position].second<=numCameras_,
                       "Completion counter is larger than the amount of cameras in the system!");
  }
  return found;
}

// This will return true if the internal counter on how many times detectionEndedForMultiFrame()
// has been called for this multiframe equals the number of cameras in the system.
bool FrameSynchronizer::detectionCompletedForAllCameras(uint64_t multiFrameId) {
  int position;
  if(findFrameById(multiFrameId,position)) {
    if(frameBuffer_[position].second == numCameras_) {
      OKVIS_ASSERT_TRUE(Exception,frameBuffer_[position].first->timestamp() > lastCompletedFrameTimestamp_
                            && (lastCompletedFrameId_==0 || frameBuffer_[position].first->id() > lastCompletedFrameId_) ,
                     "wrong order!\ntimestamp last: " << lastCompletedFrameTimestamp_
                     << "\ntimestamp new:  " << frameBuffer_[position].first->timestamp()
                     << "\nid last: " << lastCompletedFrameId_
                     << "\nid new:  " << frameBuffer_[position].first->id());
      lastCompletedFrameId_ = frameBuffer_[position].first->id();
      lastCompletedFrameTimestamp_ = frameBuffer_[position].first->timestamp();
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

// Find a multiframe in the buffer that has a timestamp within the tolerances of the given one. The tolerance
// is given as a parameter in okvis::VioParameters::sensors_information::frameTimestampTolerance
bool FrameSynchronizer::findFrameByTime(const okvis::Time& timestamp, int& position) const{
  bool found = false;
  for(int i=0; i < max_frame_sync_buffer_size; ++i) {
    position = (bufferPosition_+i)%max_frame_sync_buffer_size;
    if(frameBuffer_[position].first != nullptr &&
       (frameBuffer_[position].first->timestamp() == timestamp ||
        fabs((frameBuffer_[position].first->timestamp()-timestamp).toSec()) < timeTol_)) {
      found = true;
      break;
    }
  }
  return found;
}

// Find a multiframe in the buffer for a given multiframe ID.
bool FrameSynchronizer::findFrameById(uint64_t mfId, int& position) const {
  bool found = false;
  for(int i=0; i < max_frame_sync_buffer_size; ++i) {
    position = (bufferPosition_+i)%max_frame_sync_buffer_size;
    if(frameBuffer_[position].first != nullptr &&
       frameBuffer_[position].first->id() == mfId) {
      found = true;
      break;
    }
  }
  return found;
}


} /* namespace okvis */
