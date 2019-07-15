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
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis/Estimator.hpp
 * @brief Header file for the Estimator class. This does all the backend work.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_ESTIMATOR_HPP_
#define INCLUDE_OKVIS_ESTIMATOR_HPP_

#include <memory>
#include <mutex>
#include <array>

#include <ceres/ceres.h>
#include <okvis/kinematics/Transformation.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/VioBackendInterface.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Variables.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/MarginalizationError.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/CeresIterationCallback.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

//! The estimator class
/*!
 The estimator class. This does all the backend work.
 Frames:
 W: World
 B: Body
 C: Camera
 S: Sensor (IMU)
 */
class Estimator : public VioBackendInterface
{
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief The default constructor.
   */
  Estimator();

  /**
   * @brief Constructor if a ceres map is already available.
   * @param mapPtr Shared pointer to ceres map.
   */
  Estimator(std::shared_ptr<okvis::ceres::Map> mapPtr);
  virtual ~Estimator();

  /// @name Sensor configuration related
  ///@{
  /**
   * @brief Add a camera to the configuration. Sensors can only be added and never removed.
   * @param extrinsicsEstimationParameters The parameters that tell how to estimate extrinsics.
   * @return Index of new camera.
   */
  int addCamera(
      const okvis::ExtrinsicsEstimationParameters & extrinsicsEstimationParameters);

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imuParameters The IMU parameters.
   * @return index of IMU.
   */
  int addImu(const okvis::ImuParameters & imuParameters);

  /**
   * @brief Remove all cameras from the configuration
   */
  void clearCameras();

  /**
   * @brief Remove all IMUs from the configuration.
   */
  void clearImus();

  /// @}

  /**
   * @brief Add a pose to the state.
   * @param multiFrame Matched multiFrame.
   * @param imuMeasurements IMU measurements from last state to new one.
   * @param asKeyframe Is this new frame a keyframe?
   * @return True if successful.
   */
  bool addStates(okvis::MultiFramePtr multiFrame,
                 const okvis::ImuMeasurementDeque & imuMeasurements,
                 bool asKeyframe);

  /**
   * @brief Prints state information to buffer.
   * @param poseId The pose Id for which to print.
   * @param buffer The puffer to print into.
   */
  void printStates(uint64_t poseId, std::ostream & buffer) const;

  /**
   * @brief Add a landmark.
   * @param landmarkId ID of the new landmark.
   * @param landmark Homogeneous coordinates of landmark in W-frame.
   * @return True if successful.
   */
  bool addLandmark(uint64_t landmarkId,
                   const Eigen::Vector4d & landmark);

  /**
   * @brief Add an observation to a landmark.
   * \tparam GEOMETRY_TYPE The camera geometry type for this observation.
   * @param landmarkId ID of landmark.
   * @param poseId ID of pose where the landmark was observed.
   * @param camIdx ID of camera frame where the landmark was observed.
   * @param keypointIdx ID of keypoint corresponding to the landmark.
   * @return Residual block ID for that observation.
   */
  template<class GEOMETRY_TYPE>
  ::ceres::ResidualBlockId addObservation(uint64_t landmarkId, uint64_t poseId,
                                          size_t camIdx, size_t keypointIdx);
  /**
   * @brief Remove an observation from a landmark, if available.
   * @param landmarkId ID of landmark.
   * @param poseId ID of pose where the landmark was observed.
   * @param camIdx ID of camera frame where the landmark was observed.
   * @param keypointIdx ID of keypoint corresponding to the landmark.
   * @return True if observation was present and successfully removed.
   */
  bool removeObservation(uint64_t landmarkId, uint64_t poseId,  size_t camIdx,
                         size_t keypointIdx);

  /**
   * @brief Applies the dropping/marginalization strategy according to the RSS'13/IJRR'14 paper.
   *        The new number of frames in the window will be numKeyframes+numImuFrames.
   * @param numKeyframes Number of keyframes.
   * @param numImuFrames Number of frames in IMU window.
   * @param removedLandmarks Get the landmarks that were removed by this operation.
   * @return True if successful.
   */
  bool applyMarginalizationStrategy(size_t numKeyframes, size_t numImuFrames,
                                    okvis::MapPointVector& removedLandmarks);

  /**
   * @brief Initialise pose from IMU measurements. For convenience as static.
   * @param[in]  imuMeasurements The IMU measurements to be used for this.
   * @param[out] T_WS initialised pose.
   * @return True if successful.
   */
  static bool initPoseFromImu(
      const okvis::ImuMeasurementDeque & imuMeasurements,
      okvis::kinematics::Transformation & T_WS);

  /**
   * @brief Start ceres optimization.
   * @param[in] numIter Maximum number of iterations.
   * @param[in] numThreads Number of threads.
   * @param[in] verbose Print out optimization progress and result, if true.
   */
  void optimize(size_t numIter, size_t numThreads = 1, bool verbose = false);

  /**
   * @brief Set a time limit for the optimization process.
   * @param[in] timeLimit Time limit in seconds. If timeLimit < 0 the time limit is removed.
   * @param[in] minIterations minimum iterations the optimization process should do
   *            disregarding the time limit.
   * @return True if successful.
   */
  bool setOptimizationTimeLimit(double timeLimit, int minIterations);

  /**
   * @brief Checks whether the landmark is added to the estimator.
   * @param landmarkId The ID.
   * @return True if added.
   */
  bool isLandmarkAdded(uint64_t landmarkId) const {
    bool isAdded = landmarksMap_.find(landmarkId) != landmarksMap_.end();
    OKVIS_ASSERT_TRUE_DBG(Exception, isAdded == mapPtr_->parameterBlockExists(landmarkId),
                   "id="<<landmarkId<<" inconsistent. isAdded = " << isAdded);
    return isAdded;
  }

  /**
   * @brief Checks whether the landmark is initialized.
   * @param landmarkId The ID.
   * @return True if initialised.
   */
  bool isLandmarkInitialized(uint64_t landmarkId) const;

  /// @name Getters
  ///\{
  /**
   * @brief Get a specific landmark.
   * @param[in]  landmarkId ID of desired landmark.
   * @param[out] mapPoint Landmark information, such as quality, coordinates etc.
   * @return True if successful.
   */
  bool getLandmark(uint64_t landmarkId, okvis::MapPoint& mapPoint) const;

  /**
   * @brief Get a copy of all the landmarks as a PointMap.
   * @param[out] landmarks The landmarks.
   * @return number of landmarks.
   */
  size_t getLandmarks(okvis::PointMap & landmarks) const;

  /**
   * @brief Get a copy of all the landmark in a MapPointVector. This is for legacy support.
   *        Use getLandmarks(okvis::PointMap&) if possible.
   * @param[out] landmarks A vector of all landmarks.
   * @see getLandmarks().
   * @return number of landmarks.
   */
  size_t getLandmarks(okvis::MapPointVector & landmarks) const;

  /**
   * @brief Get a multiframe.
   * @param frameId ID of desired multiframe.
   * @return Shared pointer to multiframe.
   */
  okvis::MultiFramePtr multiFrame(uint64_t frameId) const {
    OKVIS_ASSERT_TRUE_DBG(Exception, multiFramePtrMap_.find(frameId)!=multiFramePtrMap_.end(),
                       "Requested multi-frame does not exist in estimator.");
    return multiFramePtrMap_.at(frameId);
  }

  /**
   * @brief Get pose for a given pose ID.
   * @param[in]  poseId ID of desired pose.
   * @param[out] T_WS Homogeneous transformation of this pose.
   * @return True if successful.
   */
  bool get_T_WS(uint64_t poseId, okvis::kinematics::Transformation & T_WS) const;

  // the following access the optimization graph, so are not very fast.
  // Feel free to implement caching for them...
  /**
   * @brief Get speeds and IMU biases for a given pose ID.
   * @warning This accesses the optimization graph, so not very fast.
   * @param[in]  poseId ID of pose to get speeds and biases for.
   * @param[in]  imuIdx index of IMU to get biases for. As only one IMU is supported this is always 0.
   * @param[out] speedAndBias Speed And bias requested.
   * @return True if successful.
   */
  bool getSpeedAndBias(uint64_t poseId, uint64_t imuIdx, okvis::SpeedAndBias & speedAndBias) const;

  /**
   * @brief Get camera states for a given pose ID.
   * @warning This accesses the optimization graph, so not very fast.
   * @param[in]  poseId ID of pose to get camera state for.
   * @param[in]  cameraIdx index of camera to get state for.
   * @param[out] T_SCi Homogeneous transformation from sensor (IMU) frame to camera frame.
   * @return True if successful.
   */
  bool getCameraSensorStates(uint64_t poseId, size_t cameraIdx,
                              okvis::kinematics::Transformation & T_SCi) const;

  /// @brief Get the number of states/frames in the estimator.
  /// \return The number of frames.
  size_t numFrames() const {
    return statesMap_.size();
  }

  /// @brief Get the number of landmarks in the estimator
  /// \return The number of landmarks.
  size_t numLandmarks() const {
    return landmarksMap_.size();
  }

  /// @brief Get the ID of the current keyframe.
  /// \return The ID of the current keyframe.
  uint64_t currentKeyframeId() const;

  /**
   * @brief Get the ID of an older frame.
   * @param[in] age age of desired frame. 0 would be the newest frame added to the state.
   * @return ID of the desired frame or 0 if parameter age was out of range.
   */
  uint64_t frameIdByAge(size_t age) const;

  /// @brief Get the ID of the newest frame added to the state.
  /// \return The ID of the current frame.
  uint64_t currentFrameId() const;

  ///@}

  /**
   * @brief Checks if a particular frame is a keyframe.
   * @param[in] frameId ID of frame to check.
   * @return True if the frame is a keyframe.
   */
  bool isKeyframe(uint64_t frameId) const {
    return statesMap_.at(frameId).isKeyframe;
  }

  /**
   * @brief Checks if a particular frame is still in the IMU window.
   * @param[in] frameId ID of frame to check.
   * @return True if the frame is in IMU window.
   */
  bool isInImuWindow(uint64_t frameId) const;

  /// @name Getters
  /// @{
  /**
   * @brief Get the timestamp for a particular frame.
   * @param[in] frameId ID of frame.
   * @return Timestamp of frame.
   */
  okvis::Time timestamp(uint64_t frameId) const {
    return statesMap_.at(frameId).timestamp;
  }

  ///@}
  /// @name Setters
  ///@{
  /**
   * @brief Set pose for a given pose ID.
   * @warning This accesses the optimization graph, so not very fast.
   * @param[in] poseId ID of the pose that should be changed.
   * @param[in] T_WS new homogeneous transformation.
   * @return True if successful.
   */
  bool set_T_WS(uint64_t poseId, const okvis::kinematics::Transformation & T_WS);

  /**
   * @brief Set the speeds and IMU biases for a given pose ID.
   * @warning This accesses the optimization graph, so not very fast.
   * @param[in] poseId ID of the pose to change corresponding speeds and biases for.
   * @param[in] imuIdx index of IMU to get biases for. As only one IMU is supported this is always 0.
   * @param[in] speedAndBias new speeds and biases.
   * @return True if successful.
   */
  bool setSpeedAndBias(uint64_t poseId, size_t imuIdx, const okvis::SpeedAndBias & speedAndBias);

  /**
   * @brief Set the transformation from sensor to camera frame for a given pose ID.
   * @warning This accesses the optimization graph, so not very fast.
   * @param[in] poseId ID of the pose to change corresponding camera states for.
   * @param[in] cameraIdx Index of camera to set state for.
   * @param[in] T_SCi new homogeneous transformation from sensor (IMU) to camera frame.
   * @return True if successful.
   */
  bool setCameraSensorStates(uint64_t poseId, size_t cameraIdx,
                              const okvis::kinematics::Transformation & T_SCi);

  /// @brief Set the homogeneous coordinates for a landmark.
  /// @param[in] landmarkId The landmark ID.
  /// @param[in] landmark Homogeneous coordinates of landmark in W-frame.
  /// @return True if successful.
  bool setLandmark(uint64_t landmarkId, const Eigen::Vector4d & landmark);

  /// @brief Set the landmark initialization state.
  /// @param[in] landmarkId The landmark ID.
  /// @param[in] initialized Whether or not initialised.
  void setLandmarkInitialized(uint64_t landmarkId, bool initialized);

  /// @brief Set whether a frame is a keyframe or not.
  /// @param[in] frameId The frame ID.
  /// @param[in] isKeyframe Whether or not keyrame.
  void setKeyframe(uint64_t frameId, bool isKeyframe){
    statesMap_.at(frameId).isKeyframe = isKeyframe;
  }

  /// @brief set ceres map
  /// @param[in] mapPtr The pointer to the okvis::ceres::Map.
  void setMap(std::shared_ptr<okvis::ceres::Map> mapPtr) {
    mapPtr_ = mapPtr;
  }
  ///@}

 private:

  /**
   * @brief Remove an observation from a landmark.
   * @param residualBlockId Residual ID for this landmark.
   * @return True if successful.
   */
  bool removeObservation(::ceres::ResidualBlockId residualBlockId);

  /// \brief StateInfo This configures the state vector ordering
  struct StateInfo
  {
    /// \brief Constructor
    /// @param[in] id The Id.
    /// @param[in] isRequired Whether or not we require the state.
    /// @param[in] exists Whether or not this exists in the ceres problem.
    StateInfo(uint64_t id = 0, bool isRequired = true, bool exists = false)
        : id(id),
          isRequired(isRequired),
          exists(exists)
    {
    }
    uint64_t id; ///< The ID.
    bool isRequired; ///< Whether or not we require the state.
    bool exists; ///< Whether or not this exists in the ceres problem.
  };

  /// \brief GlobalStates The global states enumerated
  enum GlobalStates
  {
    T_WS = 0, ///< Pose.
    MagneticZBias = 1, ///< Magnetometer z-bias, currently unused
    Qff = 2, ///< QFF (pressure at sea level), currently unused
    T_GW = 3, ///< Alignment of global frame, currently unused
  };

  /// \brief SensorStates The sensor-internal states enumerated
  enum SensorStates
  {
    Camera = 0, ///< Camera
    Imu = 1, ///< IMU
    Position = 2, ///< Position, currently unused
    Gps = 3, ///< GPS, currently unused
    Magnetometer = 4, ///< Magnetometer, currently unused
    StaticPressure = 5, ///< Static pressure, currently unused
    DynamicPressure = 6 ///< Dynamic pressure, currently unused
  };

  /// \brief CameraSensorStates The camera-internal states enumerated
  enum CameraSensorStates
  {
    T_SCi = 0, ///< Extrinsics as T_SC
    Intrinsics = 1, ///< Intrinsics
  };

  /// \brief ImuSensorStates The IMU-internal states enumerated
  /// \warning This is slightly inconsistent, since the velocity should be global.
  enum ImuSensorStates
  {
    SpeedAndBias = 0 ///< Speed and biases as v in S-frame, then b_g and b_a
  };

  /// \brief PositionSensorStates, currently unused
  enum PositionSensorStates
  {
    T_PiW = 0,  ///< position sensor frame to world, currently unused
    PositionSensorB_t_BA = 1  ///< antenna offset, currently unused
  };

  /// \brief GpsSensorStates, currently unused
  enum GpsSensorStates
  {
    GpsB_t_BA = 0  ///< antenna offset, currently unused
  };

  /// \brief MagnetometerSensorStates, currently unused
  enum MagnetometerSensorStates
  {
    MagnetometerBias = 0 ///< currently unused
  };

  /// \brief GpsSensorStates, currently unused
  enum StaticPressureSensorStates
  {
    StaticPressureBias = 0 ///< currently unused
  };

  /// \brief GpsSensorStates, currently unused
  enum DynamicPressureSensorStates
  {
    DynamicPressureBias = 0 ///< currently unused
  };

  // getters
  bool getGlobalStateParameterBlockPtr(uint64_t poseId, int stateType,
                                    std::shared_ptr<ceres::ParameterBlock>& stateParameterBlockPtr) const;
  template<class PARAMETER_BLOCK_T>
  bool getGlobalStateParameterBlockAs(uint64_t poseId, int stateType,
                                      PARAMETER_BLOCK_T & stateParameterBlock) const;
  template<class PARAMETER_BLOCK_T>
  bool getGlobalStateEstimateAs(uint64_t poseId, int stateType,
                                typename PARAMETER_BLOCK_T::estimate_t & state) const;

  bool getSensorStateParameterBlockPtr(uint64_t poseId, int sensorIdx,
                                    int sensorType, int stateType,
                                    std::shared_ptr<ceres::ParameterBlock>& stateParameterBlockPtr) const;
  template<class PARAMETER_BLOCK_T>
  bool getSensorStateParameterBlockAs(uint64_t poseId, int sensorIdx,
                                      int sensorType, int stateType,
                                      PARAMETER_BLOCK_T & stateParameterBlock) const;
  template<class PARAMETER_BLOCK_T>
  bool getSensorStateEstimateAs(uint64_t poseId, int sensorIdx, int sensorType,
                                int stateType, typename PARAMETER_BLOCK_T::estimate_t & state) const;

  // setters
  template<class PARAMETER_BLOCK_T>
  bool setGlobalStateEstimateAs(uint64_t poseId, int stateType,
                                const typename PARAMETER_BLOCK_T::estimate_t & state);
  template<class PARAMETER_BLOCK_T>
  bool setSensorStateEstimateAs(uint64_t poseId, int sensorIdx, int sensorType,
                                int stateType, const typename PARAMETER_BLOCK_T::estimate_t & state);

  // the following are just fixed-size containers for related parameterBlockIds:
  typedef std::array<StateInfo, 6> GlobalStatesContainer; ///< Container for global states.
  typedef std::vector<StateInfo> SpecificSensorStatesContainer;  ///< Container for sensor states. The dimension can vary from sensor to sensor...
  typedef std::array<std::vector<SpecificSensorStatesContainer>, 7> AllSensorStatesContainer; ///< Union of all sensor states.

  /// \brief States This summarizes all the possible states -- i.e. their ids:
  struct States
  {
    States() : isKeyframe(false), id(0) {}
    States(bool isKeyframe, uint64_t id, okvis::Time timestamp)
      : isKeyframe(isKeyframe), id(id), timestamp(timestamp) {}
    GlobalStatesContainer global;
    AllSensorStatesContainer sensors;
    bool isKeyframe;
    uint64_t id;
    okvis::Time timestamp;
  };

  // the following keeps track of all the states at different time instances (key=poseId)
  std::map<uint64_t, States> statesMap_; ///< Buffer for currently considered states.
  std::map<uint64_t, okvis::MultiFramePtr> multiFramePtrMap_; ///< remember all needed okvis::MultiFrame.
  std::shared_ptr<okvis::ceres::Map> mapPtr_; ///< The underlying okvis::Map.

  // this is the reference pose
  uint64_t referencePoseId_; ///< The pose ID of the reference (currently not changing)

  // the following are updated after the optimization
  okvis::PointMap landmarksMap_; ///< Contains all the current landmarks (synched after optimisation).
  mutable std::mutex statesMutex_;  ///< Regulate access of landmarksMap_.

  // parameters
  std::vector<okvis::ExtrinsicsEstimationParameters,
      Eigen::aligned_allocator<okvis::ExtrinsicsEstimationParameters> > extrinsicsEstimationParametersVec_; ///< Extrinsics parameters.
  std::vector<okvis::ImuParameters, Eigen::aligned_allocator<okvis::ImuParameters> > imuParametersVec_; ///< IMU parameters.

  // loss function for reprojection errors
  std::shared_ptr< ::ceres::LossFunction> cauchyLossFunctionPtr_; ///< Cauchy loss.
  std::shared_ptr< ::ceres::LossFunction> huberLossFunctionPtr_; ///< Huber loss.

  // the marginalized error term
  std::shared_ptr<ceres::MarginalizationError> marginalizationErrorPtr_; ///< The marginalisation class
  ::ceres::ResidualBlockId marginalizationResidualId_; ///< Remembers the marginalisation object's Id

  // ceres iteration callback object
  std::unique_ptr<okvis::ceres::CeresIterationCallback> ceresCallback_; ///< Maybe there was a callback registered, store it here.
};

}  // namespace okvis

#include "implementation/Estimator.hpp"

#endif /* INCLUDE_OKVIS_ESTIMATOR_HPP_ */
