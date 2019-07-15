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
 *  Created on: Mar 23, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Subscriber.hpp
 * @brief Header file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_SUBSCRIBER_HPP_
#define INCLUDE_OKVIS_SUBSCRIBER_HPP_

#include <deque>
#include <memory>

#include <boost/shared_ptr.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <okvis_ros/CameraConfig.h> // generated
#pragma GCC diagnostic pop
#include <image_transport/image_transport.h>
#include "sensor_msgs/Imu.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <Eigen/Core>

#ifdef HAVE_LIBVISENSOR
  #include <visensor/visensor_api.hpp>
#endif

#include <okvis/Time.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/VioInterface.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/Publisher.hpp>
#include <okvis/VioParametersReader.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class handles all the buffering of incoming data.
 */
class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  ~Subscriber();
  /**
   * @brief Constructor. This will either subscribe to the relevant ROS topics or
   *        start up the sensor and register the callbacks directly there.
   * @param nh The ROS node handle.
   * @param vioInterfacePtr Pointer to the VioInterface.
   * @param param_reader  Parameter reader.
   */
  Subscriber(ros::NodeHandle& nh, okvis::VioInterface* vioInterfacePtr,
             const okvis::VioParametersReader& param_reader);

  /// @brief Set the node handle. This sets up the callbacks. This is called in the constructor.
  void setNodeHandle(ros::NodeHandle& nh);

 protected:
  /// @name ROS callbacks
  /// @{

  /// @brief The image callback.
  void imageCallback(const sensor_msgs::ImageConstPtr& msg,/*
   const sensor_msgs::CameraInfoConstPtr& info,*/
                     unsigned int cameraIndex);
  /// @brief The depth image callback.
  /// @warning Not implemented.
  void depthImageCallback(const sensor_msgs::ImageConstPtr& /*msg*/,
                          unsigned int /*cameraIndex*/) {
    OKVIS_THROW(Exception, "Subscriber::depthImageCallback() is not implemented.");
  }

  /// @brief The IMU callback.
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  /// @}
  /// @name Direct (no ROS) callbacks and other sensor related methods.
  /// @{

#ifdef HAVE_LIBVISENSOR
  /// @brief Initialise callbacks. Called in constructor.
  void initialiseDriverCallbacks();
  /// @brief Start up sensor.
  /// @warning Call initialiseDriverCallbacks() first to initialise sensor API.
  void startSensors(const std::vector<unsigned int>& camRate,
                    const unsigned int imuRate);
  /// @brief The IMU callback.
  void directImuCallback(boost::shared_ptr<visensor::ViImuMsg> imu_ptr,
                         visensor::ViErrorCode error);
  /// @brief The image callback.
  void directFrameCallback(visensor::ViFrame::Ptr frame_ptr,
                           visensor::ViErrorCode error);
  /// @brief The callback for images including detected corners.
  /// @warning Not implemented.
  void directFrameCornerCallback(visensor::ViFrame::Ptr frame_ptr,
                                 visensor::ViCorner::Ptr corners_ptr);
  /// \brief Dynamic reconfigure callback
  void configCallback(okvis_ros::CameraConfig &config, uint32_t level);
#endif

  /// @}
  /// @name Node and subscriber related
  /// @{

  ros::NodeHandle* nh_; ///< The node handle.
  image_transport::ImageTransport* imgTransport_; ///< The image transporter.
  std::vector<image_transport::Subscriber> imageSubscribers_; ///< The image message subscriber.
  ros::Subscriber subImu_;  ///< The IMU message subscriber.

  /// @}

#ifdef HAVE_LIBVISENSOR
  std::shared_ptr<visensor::ViSensorDriver> sensor_; ///< The sensor API.
  dynamic_reconfigure::Server<okvis_ros::CameraConfig> cameraConfigReconfigureService_; ///< dynamic reconfigure service.
#endif

  okvis::VioInterface* vioInterface_;   ///< The VioInterface. (E.g. ThreadedKFVio)
  okvis::VioParameters vioParameters_;  ///< The parameters and settings.
};
}

#endif /* INCLUDE_OKVIS_SUBSCRIBER_HPP_ */
