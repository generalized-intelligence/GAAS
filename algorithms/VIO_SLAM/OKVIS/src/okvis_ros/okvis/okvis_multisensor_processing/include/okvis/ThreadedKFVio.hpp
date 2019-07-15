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
 * @file ThreadedKFVio.hpp
 * @brief Header file for the ThreadedKFVio class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_THREADEDKFVIO_HPP_
#define INCLUDE_OKVIS_THREADEDKFVIO_HPP_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Frontend.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/assert_macros.hpp>

#include <okvis/ImuFrameSynchronizer.hpp>
#include <okvis/FrameSynchronizer.hpp>
#include <okvis/VioVisualizer.hpp>
#include <okvis/timing/Timer.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>

#ifdef USE_MOCK
#include <../test/MockVioFrontendInterface.hpp>
#include <../test/MockVioBackendInterface.hpp>
#else
#include <okvis/Estimator.hpp>
#include <okvis/VioFrontendInterface.hpp>
#endif

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 *  \brief
 *  This class manages the complete data flow in and out of the algorithm, as well as
 *  between the processing threads.
 *
 *  To ensure fast return from user callbacks, new data are collected in thread save
 *  input queues and are processed in data consumer threads, one for each data source.
 *  The algorithm tasks are running in individual threads with thread save queue for
 *  message passing.
 *  For sending back data to the user, publisher threads are created for each output
 *  which ensure that the algorithm is not being blocked by slow users.
 *
 *  All the queues can be limited to size 1 to back propagate processing congestions
 *  to the user callback.
 */
class ThreadedKFVio : public VioInterface {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

#ifdef DEACTIVATE_TIMERS
  typedef okvis::timing::DummyTimer TimerSwitchable;
#else
  typedef okvis::timing::Timer TimerSwitchable;
#endif

#ifdef USE_MOCK

  /// \brief constructor for gmock
  ThreadedKFVio(okvis::VioParameters& parameters, okvis::MockVioBackendInterface& estimator,
      okvis::MockVioFrontendInterface& frontend);

#else
  /**
   * \brief Constructor.
   * \param parameters Parameters and settings.
   */
  ThreadedKFVio(okvis::VioParameters& parameters);
#endif

  /// \brief Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
  virtual ~ThreadedKFVio();

  /// \name Add measurements to the algorithm
  /// \{
  /**
   * \brief              Add a new image.
   * \param stamp        The image timestamp.
   * \param cameraIndex  The index of the camera that the image originates from.
   * \param image        The image.
   * \param keypoints    Optionally aready pass keypoints. This will skip the detection part.
   * \param asKeyframe   Use the new image as keyframe. Not implemented.
   * \warning The frame consumer loop does not support using existing keypoints yet.
   * \warning Already specifying whether this frame should be a keyframe is not implemented yet.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addImage(const okvis::Time & stamp, size_t cameraIndex,
                        const cv::Mat & image,
                        const std::vector<cv::KeyPoint> * keypoints = 0,
                        bool* asKeyframe = 0);

  /**
   * \brief             Add an abstracted image observation.
   * \warning Not implemented.
   * \param stamp       The timestamp for the start of integration time for the image.
   * \param cameraIndex The index of the camera.
   * \param keypoints   A vector where each entry represents a [u,v] keypoint measurement. Also set the size field.
   * \param landmarkIds A vector of landmark ids for each keypoint measurement.
   * \param descriptors A matrix containing the descriptors for each keypoint.
   * \param asKeyframe  Optionally force keyframe or not.
   * \return            Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addKeypoints(const okvis::Time & stamp, size_t cameraIndex,
                            const std::vector<cv::KeyPoint> & keypoints,
                            const std::vector<uint64_t> & landmarkIds,
                            const cv::Mat& descriptors = cv::Mat(),
                            bool* asKeyframe = 0);

  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return Returns true normally. False if the previous one has not been processed yet.
   */
  virtual bool addImuMeasurement(const okvis::Time & stamp,
                                 const Eigen::Vector3d & alpha,
                                 const Eigen::Vector3d & omega);

  /**
   * \brief                      Add a position measurement.
   * \warning Not implemented.
   * \param stamp                The measurement timestamp.
   * \param position             The position in world frame.
   * \param positionOffset       Body frame antenna position offset [m].
   * \param positionCovariance   The position measurement covariance matrix.
   */
  virtual void addPositionMeasurement(
      const okvis::Time & stamp, const Eigen::Vector3d & position,
      const Eigen::Vector3d & positionOffset,
      const Eigen::Matrix3d & positionCovariance);

  /**
   * \brief                       Add a GPS measurement.
   * \warning Not implemented.
   * \param stamp                 The measurement timestamp.
   * \param lat_wgs84_deg         WGS84 latitude [deg].
   * \param lon_wgs84_deg         WGS84 longitude [deg].
   * \param alt_wgs84_deg         WGS84 altitude [m].
   * \param positionOffset        Body frame antenna position offset [m].
   * \param positionCovarianceENU The position measurement covariance matrix.
   */
  virtual void addGpsMeasurement(const okvis::Time & stamp,
                                 double lat_wgs84_deg, double lon_wgs84_deg,
                                 double alt_wgs84_deg,
                                 const Eigen::Vector3d & positionOffset,
                                 const Eigen::Matrix3d & positionCovarianceENU);

  /**
   * \brief                      Add a magnetometer measurement.
   * \warning Not implemented.
   * \param stamp                The measurement timestamp.
   * \param fluxDensityMeas      Measured magnetic flux density (sensor frame) [uT].
   * \param stdev                Measurement std deviation [uT].
   */
  virtual void addMagnetometerMeasurement(
      const okvis::Time & stamp, const Eigen::Vector3d & fluxDensityMeas,
      double stdev);

  /**
   * \brief                      Add a static pressure measurement.
   * \warning Not implemented.
   * \param stamp                The measurement timestamp.
   * \param staticPressure       Measured static pressure [Pa].
   * \param stdev                Measurement std deviation [Pa].
   */
  virtual void addBarometerMeasurement(const okvis::Time & stamp,
                                       double staticPressure, double stdev);

  /**
   * \brief                      Add a differential pressure measurement.
   * \warning Not implemented.
   * \param stamp                The measurement timestamp.
   * \param differentialPressure Measured differential pressure [Pa].
   * \param stdev                Measurement std deviation [Pa].
   */
  virtual void addDifferentialPressureMeasurement(const okvis::Time & stamp,
                                                  double differentialPressure,
                                                  double stdev);

  /// \}
  /// \name Setters
  /// \{
  /**
   * \brief Set the blocking variable that indicates whether the addMeasurement() functions
   *        should return immediately (blocking=false), or only when the processing is complete.
   */
  virtual void setBlocking(bool blocking);

  /// \}

  /// \brief Trigger display (needed because OSX won't allow threaded display).
  void display();

 private:
  /// \brief Start all threads.
  virtual void startThreads();
  /// \brief Initialises settings and calls startThreads().
  void init();

 private:

  /// \brief Loop to process frames from camera with index cameraIndex
  void frameConsumerLoop(size_t cameraIndex);
  /// \brief Loop that matches frames with existing frames.
  void matchingLoop();
  /// \brief Loop to process IMU measurements.
  void imuConsumerLoop();
  /// \brief Loop to process position measurements.
  /// \warning Not implemented.
  void positionConsumerLoop();
  /// \brief Loop to process GPS measurements.
  /// \warning Not implemented.
  void gpsConsumerLoop();
  /// \brief Loop to process magnetometer measurements.
  /// \warning Not implemented.
  void magnetometerConsumerLoop();
  /// \brief Loop to process differential pressure measurements.
  /// \warning Not implemented.
  void differentialConsumerLoop();

  /// \brief Loop that visualizes completed frames.
  void visualizationLoop();
  /// \brief Loop that performs the optimization and marginalisation.
  void optimizationLoop();
  /// \brief Loop that publishes the newest state and landmarks.
  void publisherLoop();

  /**
   * @brief Get a subset of the recorded IMU measurements.
   * @param start The first IMU measurement in the return value will be older than this timestamp.
   * @param end The last IMU measurement in the return value will be newer than this timestamp.
   * @remark This function is threadsafe.
   * @return The IMU Measurement spanning at least the time between start and end.
   */
  okvis::ImuMeasurementDeque getImuMeasurments(okvis::Time& start,
                                               okvis::Time& end);

  /**
   * @brief Remove IMU measurements from the internal buffer.
   * @param eraseUntil Remove all measurements that are strictly older than this time.
   * @return The number of IMU measurements that have been removed
   */
  int deleteImuMeasurements(const okvis::Time& eraseUntil);

 private:

  /// @brief This struct contains the results of the optimization for ease of publication.
  ///        It is also used for publishing poses that have been propagated with the IMU
  ///        measurements.
  struct OptimizationResults {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    okvis::Time stamp;                          ///< Timestamp of the optimized/propagated pose.
    okvis::kinematics::Transformation T_WS;     ///< The pose.
    okvis::SpeedAndBias speedAndBiases;         ///< The speeds and biases.
    Eigen::Matrix<double, 3, 1> omega_S;        ///< The rotational speed of the sensor.
    /// The relative transformation of the cameras to the sensor (IMU) frame
    std::vector<okvis::kinematics::Transformation,
        Eigen::aligned_allocator<okvis::kinematics::Transformation> > vector_of_T_SCi;
    okvis::MapPointVector landmarksVector;      ///< Vector containing the current landmarks.
    okvis::MapPointVector transferredLandmarks; ///< Vector of the landmarks that have been marginalized out.
    bool onlyPublishLandmarks;                  ///< Boolean to signalise the publisherLoop() that only the landmarks should be published
  };

  /// @name State variables
  /// @{

  okvis::SpeedAndBias speedAndBiases_propagated_;     ///< The speeds and IMU biases propagated by the IMU measurements.
  /// \brief The IMU parameters.
  /// \warning Duplicate of parameters_.imu
  okvis::ImuParameters imu_params_;
  okvis::kinematics::Transformation T_WS_propagated_; ///< The pose propagated by the IMU measurements
  std::shared_ptr<okvis::MapPointVector> map_;        ///< The map. Unused.

  // lock lastState_mutex_ when accessing these
  /// \brief Resulting pose of the last optimization
  /// \warning Lock lastState_mutex_.
  okvis::kinematics::Transformation lastOptimized_T_WS_;
  /// \brief Resulting speeds and IMU biases after last optimization.
  /// \warning Lock lastState_mutex_.
  okvis::SpeedAndBias lastOptimizedSpeedAndBiases_;
  /// \brief Timestamp of newest frame used in the last optimization.
  /// \warning Lock lastState_mutex_.
  okvis::Time lastOptimizedStateTimestamp_;
  /// This is set to true after optimization to signal the IMU consumer loop to repropagate
  /// the state from the lastOptimizedStateTimestamp_.
  std::atomic_bool repropagationNeeded_;

  /// @}

  ImuFrameSynchronizer imuFrameSynchronizer_;  ///< The IMU frame synchronizer.
  /// \brief The frame synchronizer responsible for merging frames into multiframes
  /// \warning Lock with frameSynchronizer_mutex_
  okvis::FrameSynchronizer frameSynchronizer_;

  okvis::Time lastAddedStateTimestamp_; ///< Timestamp of the newest state in the Estimator.
  okvis::Time lastAddedImageTimestamp_; ///< Timestamp of the newest image added to the image input queue.


  /// @name Measurement input queues
  /// @{

  /// Camera measurement input queues. For each camera in the configuration one.
  std::vector<std::shared_ptr<
      okvis::threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::CameraMeasurement> > > > cameraMeasurementsReceived_;
  /// IMU measurement input queue.
  okvis::threadsafe::ThreadSafeQueue<okvis::ImuMeasurement> imuMeasurementsReceived_;

  /// Position measurement input queue.
  okvis::threadsafe::ThreadSafeQueue<okvis::PositionMeasurement> positionMeasurementsReceived_;

  /// @}
  /// @name Measurement operation queues.
  /// @{

  /// The queue containing multiframes with completely detected frames.
  okvis::threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::MultiFrame> > keypointMeasurements_;
  /// The queue containing multiframes with completely matched frames. These are already part of the estimator state.
  okvis::threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::MultiFrame>> matchedFrames_;
  /// \brief The IMU measurements.
  /// \warning Lock with imuMeasurements_mutex_.
  okvis::ImuMeasurementDeque imuMeasurements_;
  /// \brief The Position measurements.
  /// \warning Lock with positionMeasurements_mutex_.
  okvis::PositionMeasurementDeque positionMeasurements_;
  /// The queue containing the results of the optimization or IMU propagation ready for publishing.
  okvis::threadsafe::ThreadSafeQueue<OptimizationResults> optimizationResults_;
  /// The queue containing visualization data that is ready to be displayed.
  okvis::threadsafe::ThreadSafeQueue<VioVisualizer::VisualizationData::Ptr> visualizationData_;
  /// The queue containing the actual display images
  okvis::threadsafe::ThreadSafeQueue<std::vector<cv::Mat>> displayImages_;

  /// @}
  /// @name Mutexes
  /// @{

  std::mutex imuMeasurements_mutex_;      ///< Lock when accessing imuMeasurements_
  std::mutex positionMeasurements_mutex_;      ///< Lock when accessing imuMeasurements_
  std::mutex frameSynchronizer_mutex_;    ///< Lock when accessing the frameSynchronizer_.
  std::mutex estimator_mutex_;            ///< Lock when accessing the estimator_.
  ///< Condition variable to signalise that optimization is done.
  std::condition_variable optimizationNotification_;
  /// Boolean flag for whether optimization is done for the last state that has been added to the estimator.
  std::atomic_bool optimizationDone_;
  std::mutex lastState_mutex_;            ///< Lock when accessing any of the 'lastOptimized*' variables.

  /// @}
  /// @name Consumer threads
  /// @}

  /// Threads running the frameConsumerLoop(). One per camera.
  std::vector<std::thread> frameConsumerThreads_;
  std::vector<std::thread> keypointConsumerThreads_;  ///< Threads running matchingLoop().
  std::vector<std::thread> matchesConsumerThreads_;   ///< Unused.
  std::thread imuConsumerThread_;           ///< Thread running imuConsumerLoop().
  std::thread positionConsumerThread_;      ///< Thread running positionConsumerLoop().
  std::thread gpsConsumerThread_;           ///< Thread running gpsConsumerLoop().
  std::thread magnetometerConsumerThread_;  ///< Thread running magnetometerConsumerLoop().
  std::thread differentialConsumerThread_;  ///< Thread running differentialConsumerLoop().

  /// @}
  /// @name Algorithm threads
  /// @{

  std::thread visualizationThread_; ///< Thread running visualizationLoop().
  std::thread optimizationThread_;  ///< Thread running optimizationLoop().
  std::thread publisherThread_;     ///< Thread running publisherLoop().

  /// @}
  /// @name Algorithm objects.
  /// @{

#ifdef USE_MOCK
  okvis::MockVioBackendInterface& estimator_;
  okvis::MockVioFrontendInterface& frontend_;
#else
  okvis::Estimator estimator_;    ///< The backend estimator.
  okvis::Frontend frontend_;      ///< The frontend.
#endif

  /// @}

  size_t numCameras_;     ///< Number of cameras in the system.
  size_t numCameraPairs_; ///< Number of camera pairs in the system.

  okvis::VioParameters parameters_; ///< The parameters and settings.

  /// The maximum input queue size before IMU measurements are dropped.
  /// The maximum input queue size for the camera measurements is proportionally higher
  /// depending on the ratio between IMU and camera rate.
  const size_t maxImuInputQueueSize_;

  /// Max position measurements before dropping.
  const size_t maxPositionInputQueueSize_ = 10;
  
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_THREADEDKFVIO_HPP_ */
