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
 * @file VioInterface.cpp
 * @brief Source file for the VioInterface class.
 * @author Paul Furgale
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <fstream>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/VioInterface.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
VioInterface::VioInterface() {
}
VioInterface::~VioInterface() {
  if (csvImuFile_)
    csvImuFile_->close();
  if (csvPosFile_)
    csvPosFile_->close();
  if (csvMagFile_)
    csvMagFile_->close();
  // also close all registered tracks files
  for (FilePtrMap::iterator it = csvTracksFiles_.begin();
      it != csvTracksFiles_.end(); ++it) {
    if (it->second)
      it->second->close();
  }
}

// This is just handy for the python interface.
bool VioInterface::addEigenImage(const okvis::Time & stamp, size_t cameraIndex,
                                 const EigenImage & image) {
  cv::Mat mat8;
  cv::eigen2cv(image, mat8);
  return addImage(stamp, cameraIndex, mat8);

}

// Set the callback to be called every time a new state is estimated.
void VioInterface::setStateCallback(const StateCallback & stateCallback) {
  stateCallback_ = stateCallback;
}

// Set the fullStateCallback to be called every time a new state is estimated.
void VioInterface::setFullStateCallback(
    const FullStateCallback & fullStateCallback) {
  fullStateCallback_ = fullStateCallback;
}

// Set the callback to be called every time a new state is estimated.
void VioInterface::setFullStateCallbackWithExtrinsics(
    const FullStateCallbackWithExtrinsics & fullStateCallbackWithExtrinsics) {
  fullStateCallbackWithExtrinsics_ = fullStateCallbackWithExtrinsics;
}

// Set the fullStateCallbackWithExtrinsics to be called every time a new state is estimated.
void VioInterface::setLandmarksCallback(
    const LandmarksCallback & landmarksCallback) {
  landmarksCallback_ = landmarksCallback;
}

// Set the blocking variable that indicates whether the addMeasurement() functions
// should return immediately (blocking=false), or only when the processing is complete.
void VioInterface::setBlocking(bool blocking) {
  blocking_ = blocking;
}

// Write first line of IMU CSV file to describe columns.
bool VioInterface::writeImuCsvDescription() {
  if (!csvImuFile_)
    return false;
  if (!csvImuFile_->good())
    return false;
  *csvImuFile_ << "timestamp" << ", " << "omega_tilde_WS_S_x" << ", "
      << "omega_tilde_WS_S_y" << ", " << "omega_tilde_WS_S_z" << ", "
      << "a_tilde_WS_S_x" << ", " << "a_tilde_WS_S_y" << ", "
      << "a_tilde_WS_S_z" << std::endl;
  return true;
}

// Write first line of position CSV file to describe columns.
bool VioInterface::writePosCsvDescription() {
  if (!csvPosFile_)
    return false;
  if (!csvPosFile_->good())
    return false;
  *csvPosFile_ << "timestamp" << ", " << "pos_E" << ", " << "pos_N" << ", "
      << "pos_U" << std::endl;
  return true;
}

// Write first line of magnetometer CSV file to describe columns.
bool VioInterface::writeMagCsvDescription() {
  if (!csvMagFile_)
    return false;
  if (!csvMagFile_->good())
    return false;
  *csvMagFile_ << "timestamp" << ", " << "mag_x" << ", " << "mag_y" << ", "
      << "mag_z" << std::endl;
  return true;
}

// Write first line of tracks (data associations) CSV file to describe columns.
bool VioInterface::writeTracksCsvDescription(size_t cameraId) {
  if (!csvTracksFiles_[cameraId])
    return false;
  if (!csvTracksFiles_[cameraId]->good())
    return false;
  *csvTracksFiles_[cameraId] << "timestamp" << ", " << "landmark_id" << ", "
      << "z_tilde_x" << ", " << "z_tilde_y" << ", " << "z_tilde_stdev" << ", "
      << "descriptor" << std::endl;
  return false;
}

// Set a CVS file where the IMU data will be saved to.
bool VioInterface::setImuCsvFile(std::fstream& csvFile) {
  if (csvImuFile_) {
    csvImuFile_->close();
  }
  csvImuFile_.reset(&csvFile);
  writeImuCsvDescription();
  return csvImuFile_->good();
}

// Set a CVS file where the IMU data will be saved to.
bool VioInterface::setImuCsvFile(const std::string& csvFileName) {
  csvImuFile_.reset(
      new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeImuCsvDescription();
  return csvImuFile_->good();
}

// Set a CVS file where the position measurements will be saved to.
bool VioInterface::setPosCsvFile(std::fstream& csvFile) {
  if (csvPosFile_) {
    csvPosFile_->close();
  }
  csvPosFile_.reset(&csvFile);
  writePosCsvDescription();
  return csvPosFile_->good();
}

// Set a CVS file where the position measurements will be saved to.
bool VioInterface::setPosCsvFile(const std::string& csvFileName) {
  csvPosFile_.reset(
      new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writePosCsvDescription();
  return csvPosFile_->good();
}

// Set a CVS file where the magnetometer measurements will be saved to.
bool VioInterface::setMagCsvFile(std::fstream& csvFile) {
  if (csvMagFile_) {
    csvMagFile_->close();
  }
  csvMagFile_.reset(&csvFile);
  writeMagCsvDescription();
  return csvMagFile_->good();
}

// Set a CVS file where the magnetometer measurements will be saved to.
bool VioInterface::setMagCsvFile(const std::string& csvFileName) {
  csvMagFile_.reset(
      new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeMagCsvDescription();
  return csvMagFile_->good();
}

// Set a CVS file where the tracks (data associations) will be saved to.
bool VioInterface::setTracksCsvFile(size_t cameraId, std::fstream& csvFile) {
  if (csvTracksFiles_[cameraId]) {
    csvTracksFiles_[cameraId]->close();
  }
  csvTracksFiles_[cameraId].reset(&csvFile);
  writeTracksCsvDescription(cameraId);
  return csvTracksFiles_[cameraId]->good();
}

// Set a CVS file where the tracks (data associations) will be saved to.
bool VioInterface::setTracksCsvFile(size_t cameraId,
                                    const std::string& csvFileName) {
  csvTracksFiles_[cameraId].reset(
      new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeTracksCsvDescription(cameraId);
  return csvTracksFiles_[cameraId]->good();
}

}  // namespace okvis
