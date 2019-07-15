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
 * @brief Header file for the RosParametersReader class.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_ROSPARAMETERSREADER_HPP_
#define INCLUDE_OKVIS_ROSPARAMETERSREADER_HPP_

#include <string>

#include <okvis/VioParametersReader.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/Parameters.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <ros/callback_queue.h>

#ifdef HAVE_VISENSOR
  #ifdef USE_VISENSORNODE_V1_1 // TODO: remove this as soon as the public visensor_node gets updated!
    #include <visensor_node/visensor_calibration.h>
    #include <visensor_node/visensor_calibration_service.h>
    namespace visensor_msgs = visensor_node;
  #else
    #include <visensor_msgs/visensor_calibration.h>
    #include <visensor_msgs/visensor_calibration_service.h>
  #endif
#endif
#ifdef HAVE_LIBVISENSOR
  #include <visensor/visensor_api.hpp>
#endif

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class extends the VioParametersReader class in order to use ROS services and topics.
 */
class RosParametersReader : public VioParametersReader {
 public:

  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief The default constructor.
  RosParametersReader();

  /**
   * @brief The constructor. This calls readConfigFile().
   * @param filename Configuration filename.
   */
  RosParametersReader(const std::string& filename);

 private:

  /**
   * @brief Get the camera calibration.
   *
   *        This looks for the calibration in the
   *        configuration file first. If useDriver==false it will next try to get
   *        the calibration via the ROS service advertised by the ViSensor node.
   *        If still unsuccessful it will try to find the calibration in the
   *        ROS-topic /calibrationX. If useDriver==true it will try to directly
   *        get the calibration from the sensor.
   * @param calibrations The calibrations.
   * @param configurationFile The config file.
   * @return True if reading of the calibration was successful.
   */
  virtual bool getCameraCalibration(
      std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
      cv::FileStorage& configurationFile);

  /**
   * @brief Get the camera calibration via the ROS service advertised by the visensor node.
   * @param[out] calibrations The calibrations.
   * @return True if successful.
   */
  bool getCalibrationViaRosService(
      std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations) const;

  /**
   * @brief Get the camera calibration via the ROS topic /calibrationX.
   * @remarks Only supports visensor_msgs.
   * @param[out] calibrations The calibrations.
   * @return True if successful.
   */
  bool getCalibrationViaRosTopic(
      std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations) const;

};

}

#endif /* INCLUDE_OKVIS_ROSPARAMETERSREADER_HPP_ */
