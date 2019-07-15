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
 * @file FrameSynchronizer.hpp
 * @brief Header file for the FrameSynchronizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_FRAMESYNCHRONIZER_HPP_
#define INCLUDE_OKVIS_FRAMESYNCHRONIZER_HPP_

#include <memory>
#include <okvis/Measurements.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/VioInterface.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/assert_macros.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class combines multiple frames with the same or similar timestamp into one multiframe.
 * @warning This class is not threadsafe. Make sure to lock it with a mutex if used in multiple threads!
 */
class FrameSynchronizer {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor. Calls init().
   * @param parameters Parameters and settings.
   */
  FrameSynchronizer(okvis::VioParameters& parameters);

  /// @brief Trivial destructor.
  virtual ~FrameSynchronizer();

  /**
   * @brief Initialise the synchronizer with new parameters. Is called in the constructor.
   * @param parameters New parameters and settings.
   */
  void init(okvis::VioParameters& parameters);

  /**
   * @brief Adds a new frame to the internal buffer and returns the Multiframe containing the frame.
   * @param frame New frame.
   * @return Multiframe with the added frame in it.
   */
  std::shared_ptr<okvis::MultiFrame> addNewFrame(
      std::shared_ptr<okvis::CameraMeasurement> &frame);

  /**
   * @brief Inform the synchronizer that a frame in the multiframe has completed keypoint detection and description.
   * @warning This function does not check whether the multiframe contains newly detected keypoints and their descriptors.
   *          Therefore only call it when you are sure a frame has been processed for which you have not called this
   *          function before.
   * @param multiFrameId ID of the multiframe that contains the frame with the newly detected keypoints.
   * @return True if the multiframe was found in the synchronizer. If it returns false this means that the multiframe
   *         probably is too old and already fell out of the internal buffer.
   */
  bool detectionEndedForMultiFrame(uint64_t multiFrameId);

  /**
   * @brief This will return true if the internal counter on how many times detectionEndedForMultiFrame()
   *        has been called for this multiframe equals the number of cameras in the system.
   * @warning There is no check on whether actually all frames inside the multiframe have their keypoints detected.
   *          The synchronizer trusts the user to only ever call detectionEndedForMultiFrame() once for each frame in
   *          the multiframe.
   * @param multiFrameId  The ID of the multiframe.
   * @return True if the detection has ended for all cameras. False if either the multiframe was not found in the buffer
   *         or if not all frames in the multiframe have been detected.
   */
  bool detectionCompletedForAllCameras(uint64_t multiFrameId);

 private:

  /**
   * @brief Find a multiframe in the buffer that has a timestamp within the tolerances of the given one. The tolerance
   *        is given as a parameter in okvis::VioParameters::sensors_information::frameTimestampTolerance
   * @param[in]  timestamp Look for this timestamp in the buffer.
   * @param[out] position  Position of the found multiframe in the buffer. Check the return value first to find out
   *                       whether the multiframe was found.
   * @return True if a multiframe with a timestamp within tolerances has been found.
   */
  bool findFrameByTime(const okvis::Time& timestamp, int& position) const;

  /// returns true if a frame with multiframe id mfId is found and sets position to its frame buffer position
  /**
   * @brief Find a multiframe in the buffer for a given multiframe ID.
   * @param[in]  mfId Look for this ID in the buffer.
   * @param[out] position Position of the desired multiframe in the buffer. Check the return value to first find out
   *                      whether the multiframe was found.
   * @return True if a multiframe with the given timestamp was found in the buffer.
   */
  bool findFrameById(uint64_t mfId, int& position) const;

  /// Copy of the parameters and settings.
  okvis::VioParameters parameters_;
  /// Number of cameras for easy access.
  size_t numCameras_;
  /// Timestamp tolerance to classify multiple frames as being part of the same multiframe.
  double timeTol_;
  /// Circular buffer containing a multiframe pointer and a counter for how many times detection has completed.
  std::vector<std::pair<std::shared_ptr<okvis::MultiFrame>, size_t> > frameBuffer_;
  /// Position of the newest multiframe in the buffer.
  int bufferPosition_;

  /// Timestamp of the last multiframe that returned true in detectionCompletedForAllCameras().
  okvis::Time lastCompletedFrameTimestamp_;
  /// ID of the last multiframe that returned true in detectionCompletedForAllCameras().
  uint64_t lastCompletedFrameId_;

};

} /* namespace okvis */

#endif /* INCLUDE_OKVIS_FRAMESYNCHRONIZER_HPP_ */
