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
 *  Created on: Jun 11, 2013
 *      Author: Paul Furgale
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioInterface.hpp
 * @brief Header file for the VioInterface class.
 * @author Paul Furgale
 * @author Stefan Leutenegger
 * @author Andreas Froster
 */

#ifndef INCLUDE_OKVIS_VIOINTERFACE_HPP_
#define INCLUDE_OKVIS_VIOINTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <functional>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#pragma GCC diagnostic pop
#include <okvis/assert_macros.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief An abstract base class for interfaces between Front- and Backend.
 */
class VioInterface {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  typedef std::function<
  void(const okvis::Time &, const okvis::kinematics::Transformation &)> StateCallback;
  typedef std::function<
      void(const okvis::Time &, const okvis::kinematics::Transformation &,
           const Eigen::Matrix<double, 9, 1> &,
           const Eigen::Matrix<double, 3, 1> &)> FullStateCallback;
  typedef std::function<
      void(
          const okvis::Time &,
          const okvis::kinematics::Transformation &,
          const Eigen::Matrix<double, 9, 1> &,
          const Eigen::Matrix<double, 3, 1> &,
          const std::vector<okvis::kinematics::Transformation,
              Eigen::aligned_allocator<okvis::kinematics::Transformation> >&)> FullStateCallbackWithExtrinsics;
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> EigenImage;
  typedef std::function<
      void(const okvis::Time &, const okvis::MapPointVector &,
           const okvis::MapPointVector &)> LandmarksCallback;

  VioInterface();
  virtual ~VioInterface();

  /// \name Setters
  /// \{

  /// \brief              Set a CVS file where the IMU data will be saved to.
  /// \param csvFile      The file.
  bool setImuCsvFile(std::fstream& csvFile);
  /// \brief              Set a CVS file where the IMU data will be saved to.
  /// \param csvFileName  The filename of a new file.
  bool setImuCsvFile(const std::string& csvFileName);

  /// \brief              Set a CVS file where the tracks (data associations) will be saved to.
  /// \param cameraId     The camera ID.
  /// \param csvFile      The file.
  bool setTracksCsvFile(size_t cameraId, std::fstream& csvFile);
  /// \brief              Set a CVS file where the tracks (data associations) will be saved to.
  /// \param cameraId     The camera ID.
  /// \param csvFileName  The filename of a new file.
  bool setTracksCsvFile(size_t cameraId, const std::string& csvFileName);

  /// \brief              Set a CVS file where the position measurements will be saved to.
  /// \param csvFile      The file.
  bool setPosCsvFile(std::fstream& csvFile);
  /// \brief              Set a CVS file where the position measurements will be saved to.
  /// \param csvFileName  The filename of a new file.
  bool setPosCsvFile(const std::string& csvFileName);

  /// \brief              Set a CVS file where the magnetometer measurements will be saved to.
  /// \param csvFile      The file.
  bool setMagCsvFile(std::fstream& csvFile);
  /// \brief              Set a CVS file where the magnetometer measurements will be saved to.
  /// \param csvFileName  The filename of a new file.
  bool setMagCsvFile(const std::string& csvFileName);

  /// \}
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
                        bool* asKeyframe = 0) = 0;

  /**
   * \brief             Add an abstracted image observation.
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
                            bool* asKeyframe = 0) = 0;

  /// \brief          Add an IMU measurement.
  /// \param stamp    The measurement timestamp.
  /// \param alpha    The acceleration measured at this time.
  /// \param omega    The angular velocity measured at this time.
  virtual bool addImuMeasurement(const okvis::Time & stamp,
                                 const Eigen::Vector3d & alpha,
                                 const Eigen::Vector3d & omega) = 0;

  /// \brief                      Add a position measurement.
  /// \warning Not Implemented.
  /*
  /// \param stamp                The measurement timestamp
  /// \param position             The position in world frame
  /// \param positionCovariance   The position measurement covariance matrix.
  */
  virtual void addPositionMeasurement(
      const okvis::Time & /*stamp*/, const Eigen::Vector3d & /*position*/,
      const Eigen::Vector3d & /*positionOffset*/,
      const Eigen::Matrix3d & /*positionCovariance*/) {
    OKVIS_THROW(Exception, "not implemented");
  }

  /// \brief                       Add a position measurement.
  /// \warning Not Implemented.
  /*
  /// \param stamp                 The measurement timestamp
  /// \param lat_wgs84_deg         WGS84 latitude [deg]
  /// \param lon_wgs84_deg         WGS84 longitude [deg]
  /// \param alt_wgs84_deg         WGS84 altitude [m]
  /// \param positionOffset        Body frame antenna position offset [m]
  /// \param positionCovarianceENU The position measurement covariance matrix.
  */
  virtual void addGpsMeasurement(
      const okvis::Time & /*stamp*/, double /*lat_wgs84_deg*/,
      double /*lon_wgs84_deg*/, double /*alt_wgs84_deg*/,
      const Eigen::Vector3d & /*positionOffset*/,
      const Eigen::Matrix3d & /*positionCovarianceENU*/) {
    OKVIS_THROW(Exception, "not implemented");
  }

  /// \brief                      Add a magnetometer measurement.
  /// \warning Not Implemented.
  /*
  /// \param stamp                The measurement timestamp
  /// \param fluxDensityMeas      Measured magnetic flux density (sensor frame) [uT]
  /// \param stdev                Measurement std deviation [uT]
  */
  /// \return                     Returns true normally. False, if the previous one has not been processed yet.
  virtual void addMagnetometerMeasurement(
      const okvis::Time & /*stamp*/,
      const Eigen::Vector3d & /*fluxDensityMeas*/, double /*stdev*/) {
    OKVIS_THROW(Exception, "not implemented");
  }

  /// \brief                      Add a static pressure measurement.
  /// \warning Not Implemented.
  /*
  /// \param stamp                The measurement timestamp
  /// \param staticPressure       Measured static pressure [Pa]
  /// \param stdev                Measurement std deviation [Pa]
  */
  virtual void addBarometerMeasurement(const okvis::Time & /*stamp*/,
                                       double /*staticPressure*/,
                                       double /*stdev*/) {
    OKVIS_THROW(Exception, "not implemented");
  }

  /// \brief                      Add a differential pressure measurement.
  /// \warning Not Implemented.
  /*
  /// \param stamp                The measurement timestamp
  /// \param differentialPressure Measured differential pressure [Pa]
  /// \param stdev                Measurement std deviation [Pa]
  */
  virtual void addDifferentialPressureMeasurement(
      const okvis::Time & /*stamp*/, double /*differentialPressure*/,
      double /*stdev*/) {
    OKVIS_THROW(Exception, "not implemented");
  }

  /**
   * @brief This is just handy for the python interface.
   * @param stamp       The image timestamp
   * @param cameraIndex The index of the camera that the image originates from.
   * @param image       The image.
   * @return Returns true normally. False, if the previous one has not been processed yet.
   */
  bool addEigenImage(const okvis::Time & stamp, size_t cameraIndex,
                     const EigenImage & image);

  /// \}
  /// \name Setters
  /// \{

  /// \brief Set the stateCallback to be called every time a new state is estimated.
  ///        When an implementing class has an estimate, they can call:
  ///        stateCallback_( stamp, T_w_vk );
  ///        where stamp is the timestamp
  ///        and T_w_vk is the transformation (and uncertainty) that
  ///        transforms points from the vehicle frame to the world frame
  virtual void setStateCallback(const StateCallback & stateCallback);

  /// \brief Set the fullStateCallback to be called every time a new state is estimated.
  ///        When an implementing class has an estimate, they can call:
  ///        _fullStateCallback( stamp, T_w_vk, speedAndBiases, omega_S);
  ///        where stamp is the timestamp
  ///        and T_w_vk is the transformation (and uncertainty) that
  ///        transforms points from the vehicle frame to the world frame. speedAndBiases contain
  ///        speed in world frame followed by gyro and acc biases. finally, omega_S is the rotation speed.
  virtual void setFullStateCallback(
      const FullStateCallback & fullStateCallback);

  /// \brief Set the fullStateCallbackWithExtrinsics to be called every time a new state is estimated.
  ///        When an implementing class has an estimate, they can call:
  ///        _fullStateCallbackWithEctrinsics( stamp, T_w_vk, speedAndBiases, omega_S, vector_of_T_SCi);
  ///        where stamp is the timestamp
  ///        and T_w_vk is the transformation (and uncertainty) that
  ///        transforms points from the vehicle frame to the world frame. speedAndBiases contain
  ///        speed in world frame followed by gyro and acc biases.
  ///        omega_S is the rotation speed
  ///        vector_of_T_SCi contains the (uncertain) transformations of extrinsics T_SCi
  virtual void setFullStateCallbackWithExtrinsics(
      const FullStateCallbackWithExtrinsics & fullStateCallbackWithExtrinsics);

  /// \brief Set the landmarksCallback to be called every time a new state is estimated.
  ///        When an implementing class has an estimate, they can call:
  ///        landmarksCallback_( stamp, landmarksVector );
  ///        where stamp is the timestamp
  ///        landmarksVector contains all 3D-landmarks with id.
  virtual void setLandmarksCallback(
      const LandmarksCallback & landmarksCallback);

  /**
   * \brief Set the blocking variable that indicates whether the addMeasurement() functions
   *        should return immediately (blocking=false), or only when the processing is complete.
   */
  virtual void setBlocking(bool blocking);

  /// \}

 protected:

  /// \brief Write first line of IMU CSV file to describe columns.
  bool writeImuCsvDescription();
  /// \brief Write first line of position CSV file to describe columns.
  bool writePosCsvDescription();
  /// \brief Write first line of magnetometer CSV file to describe columns.
  bool writeMagCsvDescription();
  /// \brief Write first line of tracks (data associations) CSV file to describe columns.
  bool writeTracksCsvDescription(size_t cameraId);

  StateCallback stateCallback_; ///< State callback function.
  FullStateCallback fullStateCallback_; ///< Full state callback function.
  FullStateCallbackWithExtrinsics fullStateCallbackWithExtrinsics_; ///< Full state and extrinsics callback function.
  LandmarksCallback landmarksCallback_; ///< Landmarks callback function.
  std::shared_ptr<std::fstream> csvImuFile_;  ///< IMU CSV file.
  std::shared_ptr<std::fstream> csvPosFile_;  ///< Position CSV File.
  std::shared_ptr<std::fstream> csvMagFile_;  ///< Magnetometer CSV File
  typedef std::map<size_t, std::shared_ptr<std::fstream>> FilePtrMap;
  FilePtrMap csvTracksFiles_; ///< Tracks CSV Files.
  bool blocking_; ///< Blocking option. Whether the addMeasurement() functions should wait until proccessing is complete.
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VIOINTERFACE_HPP_ */
