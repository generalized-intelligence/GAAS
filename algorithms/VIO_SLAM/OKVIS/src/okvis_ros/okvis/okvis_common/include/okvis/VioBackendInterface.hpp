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
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioBackendInterface.hpp
 * @brief Header file for the VioBackendInterface class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_VIOBACKENDINTERFACE_HPP_
#define INCLUDE_OKVIS_VIOBACKENDINTERFACE_HPP_

#include <memory>

#include <ceres/ceres.h>

#include <okvis/assert_macros.hpp>
#include <okvis/kinematics/Transformation.hpp>

#include <okvis/MultiFrame.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/Variables.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/MarginalizationError.hpp>
#include <okvis/ceres/ReprojectionError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief An abstract interface for backends.
class VioBackendInterface {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief Default constructor.
  VioBackendInterface() {}
  virtual ~VioBackendInterface() {}

  /// @brief Enum to define the status of initialization
  enum class InitializationStatus{
    NotStarted = 0, ///< Initialization not started
    Ongoing = 1, ///< Initialization ongoing
    Complete = 2 ///< Initialization complete
  };

  /// @name Sensor configuration related
  ///@{
  /**
   * @brief Add a camera to the configuration. Sensors can only be added and never removed.
   * @param extrinsicsEstimationParameters The parameters that tell how to estimate extrinsics.
   * @return Index of new camera.
   */
  virtual int addCamera(
      const ExtrinsicsEstimationParameters & extrinsicsEstimationParameters) = 0;

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imuParameters The IMU parameters.
   * @return index of IMU.
   */
  virtual int addImu(const ImuParameters & imuParameters) = 0;

  /**
   * @brief Remove all cameras from the configuration
   */
  virtual void clearCameras() = 0;

  /**
   * @brief Remove all IMUs from the configuration.
   */
  virtual void clearImus() = 0;

  /// @}

  /**
   * @brief Add a pose to the state.
   * @param multiFrame Matched multiFrame.
   * @param imuMeasurements IMU measurements from last state to new one.
   * @param asKeyframe Is this new frame a keyframe?
   * @return True if successful.
   */
  virtual bool addStates(okvis::MultiFramePtr multiFrame,
                         const okvis::ImuMeasurementDeque & imuMeasurements,
                         bool asKeyframe) = 0;

  /**
   * @brief Add a landmark.
   * @param landmarkId ID of the new landmark.
   * @param landmark Homogeneous coordinates of landmark in W-frame.
   * @return True if successful.
   */
  virtual bool addLandmark(uint64_t landmarkId,
                           const Eigen::Vector4d & landmark) = 0;

  /**
   * @brief Remove an observation from a landmark, if available.
   * @param landmarkId ID of landmark.
   * @param poseId ID of pose where the landmark was observed.
   * @param camIdx ID of camera frame where the landmark was observed.
   * @param keypointIdx ID of keypoint corresponding to the landmark.
   * @return True if observation was present and successfully removed.
   */
  virtual bool removeObservation(uint64_t landmarkId, uint64_t poseId,  size_t camIdx,
                         size_t keypointIdx) = 0;
  /**
   * @brief Start optimization.
   * @param[in] numIter Maximum number of iterations.
   * @param[in] numThreads Number of threads.
   * @param[in] verbose Print out optimization progress and result, if true.
   */
  virtual void optimize(size_t numIter, size_t numThreads = 1, bool verbose = false) = 0;

  /**
   * @brief Set a time limit for the optimization process.
   * @param[in] timeLimit Time limit in seconds. If timeLimit < 0 the time limit is removed.
   * @param[in] minIterations minimum iterations the optimization process should do
   *            disregarding the time limit.
   * @return True if successful.
   */
  virtual bool setOptimizationTimeLimit(double timeLimit, int minIterations) = 0;

  /**
   * @brief Checks whether the landmark is added to the estimator.
   * @param landmarkId The ID.
   * @return True if added.
   */
  virtual bool isLandmarkAdded(uint64_t landmarkId) const = 0;

  /**
   * @brief Checks whether the landmark is initialized
   * @param landmarkId The ID.
   * @return True if initialised.
   */
  virtual bool isLandmarkInitialized(uint64_t landmarkId) const = 0;

  /// @name Getters
  ///\{
  /**
   * @brief Get a specific landmark.
   * @param[in]  landmarkId ID of desired landmark.
   * @param[out] mapPoint Landmark information, such as quality, coordinates etc.
   * @return True if successful.
   */
  virtual bool getLandmark(uint64_t landmarkId, MapPoint& mapPoint) const = 0;

  /**
   * @brief Get a copy of all the landmarks as a PointMap.
   * @param[out] landmarks The landmarks.
   * @return number of landmarks.
   */
  virtual size_t getLandmarks(PointMap & landmarks) const = 0; // return a copy for thread safety

  /**
   * @brief Get a multiframe.
   * @param frameId ID of desired multiframe.
   * @return Shared pointer to multiframe.
   */
  virtual okvis::MultiFramePtr multiFrame(uint64_t frameId) const = 0;

  /**
   * @brief Get pose for a given pose ID.
   * @param[in]  poseId ID of desired pose.
   * @param[out] T_WS Homogeneous transformation of this pose.
   * @return True if successful.
   */
  virtual bool get_T_WS(uint64_t poseId, okvis::kinematics::Transformation & T_WS) const = 0;

  /**
   * @brief Get speeds and IMU biases for a given pose ID.
   * @param[in]  poseId ID of pose to get speeds and biases for.
   * @param[in]  imuIdx index of IMU to get biases for.
   * @param[out] speedAndBias Speed And bias requested.
   * @return True if successful.
   */
  virtual bool getSpeedAndBias(uint64_t poseId, uint64_t imuIdx, okvis::SpeedAndBias & speedAndBias) const = 0;

  /**
   * @brief Get camera states for a given pose ID.
   * @param[in]  poseId ID of pose to get camera state for.
   * @param[in]  cameraIdx index of camera to get state for.
   * @param[out] T_SCi Homogeneous transformation from sensor (IMU) frame to camera frame.
   * @return True if successful.
   */
  virtual bool getCameraSensorStates(uint64_t poseId, size_t cameraIdx,
                              okvis::kinematics::Transformation & T_SCi) const = 0;

  /// @brief Get the number of states/frames in the estimator.
  /// \return The number of frames.
  virtual size_t numFrames() const = 0;

  /// @brief Get the number of landmarks in the backend.
  /// \return The number of landmarks.
  virtual size_t numLandmarks() const = 0;

  /// @brief Get the ID of the newest frame added to the state.
  /// \return The ID of the current frame.
  virtual uint64_t currentFrameId() const = 0;

  /// @brief Obtain the initialization status.
  virtual InitializationStatus initializationStatus() const {
  	OKVIS_THROW(Exception,"Uncertainty computation not implemented");
		return InitializationStatus::NotStarted;
  }

  ///@}
  /**
   * @brief Checks if a particular frame is a keyframe.
   * @param[in] frameId ID of frame to check.
   * @return True if the frame is a keyframe.
   */
  virtual bool isKeyframe(uint64_t frameId) const = 0;

  /// @name Getters
  /// @{
  /**
   * @brief Get the timestamp for a particular frame.
   * @param[in] frameId ID of frame.
   * @return Timestamp of frame.
   */
  virtual okvis::Time timestamp(uint64_t frameId) const = 0;

  /**
   * @brief Get the current frame pose uncertainty.
   * param[out] P_T_WS Current pose uncertainty w.r.t. [r_WS,delta_alpha_WS].
   * @return True on success.
   */
  virtual bool getPoseUncertainty(Eigen::Matrix<double,6,6> & /*P_T_WS*/) const {
    OKVIS_THROW(Exception,"Uncertainty computation not implemented");
    return false;
  }

  /**
   * @brief Get the current frame state uncertainty.
   * param[out] P_T_WS Current pose uncertainty w.r.t. [r_WS,delta_alpha_WS,v_W,b_g,b_a].
   * @return True on success.
   */
  virtual bool getStateUncertainty(Eigen::Matrix<double,15,15> & /*P*/) const {
    OKVIS_THROW(Exception,"Uncertainty computation not implemented");
    return false;
  }

  ///@}
  /// @name Setters
  ///@{
  /**
   * @brief Set pose for a given pose ID.
   * @param[in] poseId ID of the pose that should be changed.
   * @param[in] T_WS new homogeneous transformation.
   * @return True if successful.
   */
  virtual bool set_T_WS(uint64_t poseId, const okvis::kinematics::Transformation & T_WS) = 0;

  /**
   * @brief Set the speeds and IMU biases for a given pose ID.
   * @param[in] poseId ID of the pose to change corresponding speeds and biases for.
   * @param[in] imuIdx index of IMU to get biases for. As only one IMU is supported this is always 0.
   * @param[in] speedAndBias new speeds and biases.
   * @return True if successful.
   */
  virtual bool setSpeedAndBias(uint64_t poseId, size_t imuIdx, const okvis::SpeedAndBias & speedAndBias) = 0;

  /**
   * @brief Set the transformation from sensor to camera frame for a given pose ID
   * @param[in] poseId ID of the pose to change corresponding camera states for.
   * @param[in] cameraIdx Index of camera to set state for.
   * @param[in] T_SCi new homogeneous transformation from sensor (IMU) to camera frame.
   * @return True if successful.
   */
  virtual bool setCameraSensorStates(uint64_t poseId, size_t cameraIdx,
                              const okvis::kinematics::Transformation & T_SCi) = 0;

  /// @brief Set the homogeneous coordinates for a landmark
  /// @param[in] landmarkId The landmark ID.
  /// @param[in] landmark Homogeneous coordinates of landmark in W-frame.
  /// @return True if successful.
  virtual bool setLandmark(uint64_t landmarkId,
                           const Eigen::Vector4d & landmark) = 0;

  /// @brief Set the landmark initialization state
  /// @param[in] landmarkId The landmark ID.
  /// @param[in] initialized Whether or not initialised.
  virtual void setLandmarkInitialized(uint64_t landmarkId, bool initialized) = 0;

  /// @brief Set whether a frame is a keyframe or not.
  /// @param[in] frameId The frame ID.
  /// @param[in] isKeyframe Whether or not keyrame.
  virtual void setKeyframe(uint64_t frameId, bool isKeyframe) = 0;

  /// @brief Set ceres map.
  /// @param[in] mapPtr The pointer to the okvis::ceres::Map.
  virtual void setMap(std::shared_ptr<okvis::ceres::Map> mapPtr) = 0;

  /// @brief Request computation of uncertainties.
  /// param[in] computeUncertainty True, if uncertainties should be computed.
  virtual void setComputeUncertainty(bool /*computeUncertainty*/) {}

  ///@}
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VIOBACKENDINTERFACE_HPP_ */
