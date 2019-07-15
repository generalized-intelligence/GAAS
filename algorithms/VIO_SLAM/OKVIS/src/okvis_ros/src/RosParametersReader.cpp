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
 *  Created on: Jul 20, 2015
 *      Author: Andreas Forster (an.forster@gmail.com)
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file RosParametersReader.hpp
 * @brief Source file for the RosParametersReader class.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#include <glog/logging.h>

#include <okvis/RosParametersReader.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
RosParametersReader::RosParametersReader()
    : VioParametersReader() {
}

// The constructor. This calls readConfigFile().
RosParametersReader::RosParametersReader(const std::string& filename) {
  // cannot call base class constructor because it will not use the overloaded getCameraCalibration()
  readConfigFile(filename);
}

// Get the camera calibration.
bool RosParametersReader::getCameraCalibration(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileStorage& configurationFile) {

  bool success = getCalibrationViaConfig(calibrations, configurationFile["cameras"]);

  if (!useDriver && !success) {
    success = getCalibrationViaRosService(calibrations);
  }

  if (!useDriver && !success) {
    LOG(INFO) << "Could not get calibration via visensor node service.";
    success = getCalibrationViaRosTopic(calibrations);
  }

  if (!useDriver && !success)
    LOG(INFO) << "Could not get calibration via ros topic.";

#ifdef HAVE_LIBVISENSOR
  if (useDriver && !success) {
    // start up sensor
    viSensor = std::shared_ptr<visensor::ViSensorDriver>(
          new visensor::ViSensorDriver());
    try {
      // use autodiscovery to find sensor. TODO: specify IP in config?
      std::static_pointer_cast<visensor::ViSensorDriver>(viSensor)->init();
    } catch (Exception const &ex) {
      LOG(ERROR) << ex.what();
      exit(1);
    }

    success = getCalibrationViaVisensorAPI(calibrations);
  }
#endif

  return success;
}

// Get the camera calibration via the ROS service advertised by the visensor node.
bool RosParametersReader::getCalibrationViaRosService(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations) const {
#ifdef HAVE_VISENSOR
  calibrations.clear();
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<
      visensor_msgs::visensor_calibration_service>("/get_camera_calibration");
  visensor_msgs::visensor_calibration_service srv;
  const double serviceTimeout = 1.0; // seconds
  if (client.waitForExistence(ros::Duration(serviceTimeout))) {
    if (client.call(srv)) {
      LOG(INFO) << "Received calibrations for "
                << srv.response.calibration.size() << " cameras via service.";
      for (size_t i = 0; i < srv.response.calibration.size(); ++i) {
        if (srv.response.calibration[i].dist_coeff.size() < 4
            || srv.response.calibration[i].principal_point.size() < 2
            || srv.response.calibration[i].focal_length.size() < 2) {
          LOG(WARNING) << "Invalid calibration from service.";
          // remove the already received calibrations and return.
          calibrations.clear();
          return false;
        } else {
          calibrations.push_back(okvis::VioParametersReader::CameraCalibration());
#ifdef USE_VISENSORNODE_V1_1 // TODO: remove this as soon as the public visensor_node gets updated!
          geometry_msgs::Pose& T_IC = srv.response.calibration[i].T_IC;
          Eigen::Vector3d t(T_IC.position.x,
                            T_IC.position.y,
                            T_IC.position.z);
          Eigen::Quaterniond q(T_IC.orientation.w,
                               -T_IC.orientation.x,
                               -T_IC.orientation.y,
                               -T_IC.orientation.z);
          calibrations[i].T_SC = okvis::kinematics::Transformation(t,q);
          calibrations[i].imageDimension << 752, 480;
#else
          geometry_msgs::Pose& T_CI = srv.response.calibration[i].T_CI;
          Eigen::Vector3d t(T_CI.position.x,
                            T_CI.position.y,
                            T_CI.position.z);
          Eigen::Quaterniond q(T_CI.orientation.w,
                               -T_CI.orientation.x,
                               -T_CI.orientation.y,
                               -T_CI.orientation.z);
          okvis::kinematics::Transformation T_CI_okvis(t,q);
          calibrations[i].T_SC = T_CI_okvis.inverse();
          calibrations[i].imageDimension << srv.response.calibration[i].image_width,
                                            srv.response.calibration[i].image_height;
#endif
          calibrations[i].distortionCoefficients << srv.response.calibration[i].dist_coeff[0],
                                                    srv.response.calibration[i].dist_coeff[1],
                                                    srv.response.calibration[i].dist_coeff[2],
                                                    srv.response.calibration[i].dist_coeff[3];
          calibrations[i].focalLength << srv.response.calibration[i].focal_length[0],
                                         srv.response.calibration[i].focal_length[1];
          calibrations[i].principalPoint << srv.response.calibration[i].principal_point[0],
                                            srv.response.calibration[i].principal_point[1];
          calibrations[i].distortionType = srv.response.calibration[i].dist_model;
        }
      }
    }
  }
  return calibrations.empty() == false;
#else
  static_cast<void>(calibrations); // unused
  return false;
#endif // HAVE_VISENSOR
}

// Get the camera calibration via the ROS topic /calibrationX.
bool RosParametersReader::getCalibrationViaRosTopic (
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations) const {
#ifdef HAVE_VISENSOR
  calibrations.clear();
  const double topicTimeout = 0.5; // seconds
  size_t camIdx = 0;
  visensor_msgs::visensor_calibration message;
  bool receivedMessage;
  ros::NodeHandle nh;
  ros::Subscriber calibrationSub;
  bool lookForNextCameraCalibration = true;
  while (lookForNextCameraCalibration) {
    receivedMessage = false;

    // set up subscriber
    calibrationSub =
        nh.subscribe<visensor_msgs::visensor_calibration>(
          "/calibration"+std::to_string(camIdx),1,
          [camIdx, &message, &receivedMessage]
          (const visensor_msgs::visensor_calibration::ConstPtr& msg)
          {
            LOG(INFO) << "Received camera calibration for camera " << camIdx << " via topic";
            message = *msg;
            receivedMessage = true;
          });

    // spin until a timeout is reached
    ros::getGlobalCallbackQueue()->callAvailable(
          ros::WallDuration(topicTimeout));

    if (receivedMessage) {
      calibrations.push_back(okvis::VioParametersReader::CameraCalibration());
#ifdef USE_VISENSORNODE_V1_1
      geometry_msgs::Pose& T_IC = message.T_IC;
      Eigen::Vector3d t(T_IC.position.x,
                        T_IC.position.y,
                        T_IC.position.z);
      Eigen::Quaterniond q(T_IC.orientation.w,
                           -T_IC.orientation.x,
                           -T_IC.orientation.y,
                           -T_IC.orientation.z);
      calibrations[camIdx].T_SC = okvis::kinematics::Transformation(t,q);
      calibrations[camIdx].imageDimension << 752, 480;
#else
      geometry_msgs::Pose& T_CI = message.T_CI;
      Eigen::Vector3d t(T_CI.position.x,
                        T_CI.position.y,
                        T_CI.position.z);
      Eigen::Quaterniond q(T_CI.orientation.w,
                           -T_CI.orientation.x,
                           -T_CI.orientation.y,
                           -T_CI.orientation.z);
      okvis::kinematics::Transformation T_CI_okvis(t,q);
      calibrations[camIdx].T_SC = T_CI_okvis.inverse();
      calibrations[camIdx].imageDimension << message.image_width,
                                             message.image_height;
#endif
      calibrations[camIdx].distortionCoefficients << message.dist_coeff[0],
                                                     message.dist_coeff[1],
                                                     message.dist_coeff[2],
                                                     message.dist_coeff[3];
      calibrations[camIdx].focalLength << message.focal_length[0],
                                          message.focal_length[1];
      calibrations[camIdx].principalPoint << message.principal_point[0],
                                             message.principal_point[1];
      calibrations[camIdx].distortionType = message.dist_model;

      ++camIdx;
    } else
      lookForNextCameraCalibration = false;
  }

  return calibrations.empty() == false;
#else
  static_cast<void>(calibrations); // unused
  return false;
#endif // HAVE_VISENSOR
}

}  // namespace okvis
