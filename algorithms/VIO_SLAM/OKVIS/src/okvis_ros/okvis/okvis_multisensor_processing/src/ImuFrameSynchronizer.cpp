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
 *  Created on: Sep 13, 2014
 *      Author: Pascal Gohl
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ImuFrameSynchronizer.cpp
 * @brief Source file for the ImuFrameSynchronizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include "okvis/ImuFrameSynchronizer.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

ImuFrameSynchronizer::ImuFrameSynchronizer()
  : shutdown_(false) {}

ImuFrameSynchronizer::~ImuFrameSynchronizer() {
  if(!shutdown_)
    shutdown();
}

// Tell the synchronizer that a new IMU measurement has been registered.
void ImuFrameSynchronizer::gotImuData(const okvis::Time& stamp) {
  newestImuDataStamp_ = stamp;
  if(imuDataNeededUntil_ < stamp)
    gotNeededImuData_.notify_all();
}

// Wait until a IMU measurement with a timestamp equal or newer to the supplied one is registered.
bool ImuFrameSynchronizer::waitForUpToDateImuData(const okvis::Time& frame_stamp) {
  // if the newest imu data timestamp is smaller than frame_stamp, wait until
  // imu_data newer than frame_stamp arrives
  if(newestImuDataStamp_ <= frame_stamp && !shutdown_) {
    imuDataNeededUntil_ = frame_stamp;
    std::unique_lock<std::mutex> lock(mutex_);
    gotNeededImuData_.wait(lock);
  }
  if(shutdown_)
    return false;
  return true;
}

// Tell the synchronizer to shutdown. This will notify all waiting threads to wake up.
void ImuFrameSynchronizer::shutdown() {
  shutdown_ = true;
  gotNeededImuData_.notify_all();
}

} /* namespace okvis */
