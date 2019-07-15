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
 *  Created on: Mar 20, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioFrontendInterface.hpp
 * @brief Header file for the VioFrontendInterface class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_VIOFRONTENDINTERFACE_HPP_
#define INCLUDE_OKVIS_VIOFRONTENDINTERFACE_HPP_

#include <vector>
#include <memory>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <okvis/kinematics/Transformation.hpp>

#include <okvis/Measurements.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/VioBackendInterface.hpp>
#include <okvis/MultiFrame.hpp>

class Estimator;

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief The VioFrontendInterface class is an interface for frontends.
 */
class VioFrontendInterface {
 public:
  VioFrontendInterface() {
  }
  virtual ~VioFrontendInterface() {
  }
  /// @name
  /// In the derived class, the following methods (and nothing else) have to be implemented:
  ///@{
  /**
   * @brief Detection and descriptor extraction on a per image basis.
   * @param cameraIndex   Index of camera to do detection and description.
   * @param frameOut      Multiframe containing the frames.
   *                      Resulting keypoints and descriptors are saved in here.
   * @param T_WC          Pose of camera with index cameraIndex at image capture time.
   * @param[in] keypoints If the keypoints are already available from a different source, provide them here
   *                      in order to skip detection.
   * @return True if successful.
   */
  virtual bool detectAndDescribe(
      size_t cameraIndex, std::shared_ptr<okvis::MultiFrame> frameOut,
      const okvis::kinematics::Transformation& T_WC,
      const std::vector<cv::KeyPoint> * keypoints) = 0;

  /**
   * @brief Matching as well as initialization of landmarks and state.
   * @param estimator       Estimator.
   * @param T_WS_propagated Pose of sensor at image capture time.
   * @param params          Configuration parameters.
   * @param map             Current map.
   * @param framesInOut     Multiframe including the descriptors of all the keypoints.
   * @param[out] asKeyframe Should the frame be a keyframe?
   * @return True if successful.
   */
  virtual bool dataAssociationAndInitialization(
      okvis::Estimator& estimator,
      okvis::kinematics::Transformation& T_WS_propagated,
      const okvis::VioParameters & params,
      const std::shared_ptr<okvis::MapPointVector> map,
      std::shared_ptr<okvis::MultiFrame> framesInOut, bool* asKeyframe) = 0;

  /**
   * @brief Propagates pose, speeds and biases with given IMU measurements.
   * @see okvis::ceres::ImuError::propagation()
   * @param[in] imuMeasurements All the IMU measurements.
   * @param[in] imuParams The parameters to be used.
   * @param[inout] T_WS_propagated Start pose.
   * @param[inout] speedAndBiases Start speed and biases.
   * @param[in] t_start Start time.
   * @param[in] t_end End time.
   * @param[out] covariance Covariance for GIVEN start states.
   * @param[out] jacobian Jacobian w.r.t. start states.
   * @return True on success.
   */
  virtual bool propagation(const okvis::ImuMeasurementDeque & imuMeasurements,
                           const okvis::ImuParameters & imuParams,
                           okvis::kinematics::Transformation& T_WS_propagated,
                           okvis::SpeedAndBias & speedAndBiases,
                           const okvis::Time& t_start, const okvis::Time& t_end,
                           Eigen::Matrix<double, 15, 15>* covariance,
                           Eigen::Matrix<double, 15, 15>* jacobian) const = 0;

  ///@}
};

}

#endif /* INCLUDE_OKVIS_VIOFRONTENDINTERFACE_HPP_ */
