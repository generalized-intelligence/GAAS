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
 *  Created on: Sep 15, 2014
 *      Author: Pascal Gohl
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioVisualizer.hpp
 * @brief Header file for the VioVisualizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_VIOVISUALIZER_HPP_
#define INCLUDE_OKVIS_VIOVISUALIZER_HPP_

#include <opencv/highgui.h>
#include <okvis/assert_macros.hpp>

#include <okvis/Parameters.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/FrameTypedefs.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class is responsible to visualize the matching results
 */
class VioVisualizer {
 public:

  /// @brief This struct contains the relevant data for visualizing
  struct VisualizationData {
    typedef std::shared_ptr<VisualizationData> Ptr;
    okvis::ObservationVector observations;    ///< Vector containing all the keypoint observations.
    std::shared_ptr<okvis::MultiFrame> currentFrames; ///< Current multiframe.
    std::shared_ptr<okvis::MultiFrame> keyFrames;     ///< Current keyframe.
    okvis::kinematics::Transformation T_WS_keyFrame;  ///< Pose of the current keyframe
  };

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor.
   * @param parameters Parameters and settings.
   */
  VioVisualizer(okvis::VioParameters& parameters);
  virtual ~VioVisualizer();

  /**
   * @brief Initialise parameters. Called in constructor.
   * @param parameters Parameters and settings.
   */
  void init(okvis::VioParameters& parameters);
  /**
   * @brief Show the current frames with the current keyframe and all its matches.
   * @param data Visualization data containing all the info.
   */
  void showDebugImages(VisualizationData::Ptr& data);

  /**
   * @brief Circles all keypoints in the current frame, links the matching ones to
   *        the current keyframe and returns the result.
   * @param data Visualization data.
   * @param image_number Index of the frame to display.
   * @return OpenCV matrix with the resulting image.
   */
  cv::Mat drawMatches(VisualizationData::Ptr& data, size_t image_number);
  
 private:
  /**
   * @brief Circles all keypoints in the current frame and returns the result.
   * @param data Visualization data.
   * @param cameraIndex Index of the frame to display.
   * @return OpenCV matrix with the resulting image.
   */
  cv::Mat drawKeypoints(VisualizationData::Ptr& data, size_t cameraIndex);

  /// Parameters and settings.
  okvis::VioParameters parameters_;
};

} /* namespace okvis */

#endif /* INCLUDE_OKVIS_VIOVISUALIZER_HPP_ */
