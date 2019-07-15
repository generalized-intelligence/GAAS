/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Copyright (c) 2011-2013, Paul Furgale and others.
 *  All rights reserved.
 *
 *  Unlike otherwise stated in source files, the code in this repository is
 *  published under the Revised BSD (New BSD) license.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file   NsecTimeUtilities.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Sat Jul 20 12:39:54 2013
 * 
 * @brief  Functions to support the use of nanosecond epoch time.
 * 
 */
#ifndef INCLUDE_OKVIS_TIMING_NSECTIMEUTILITIES_HPP_
#define INCLUDE_OKVIS_TIMING_NSECTIMEUTILITIES_HPP_
#include <chrono>
#include <boost/cstdint.hpp>

namespace okvis {
namespace timing {

/// \brief Nanoseconds since the epoch.
typedef boost::int64_t NsecTime;

/// \brief Convert nanoseconds since the epoch to std::chrono
std::chrono::system_clock::time_point nsecToChrono( const NsecTime & time );

/// \brief Convert std::chrono to nanoseconds since the epoch.
NsecTime chronoToNsec( const std::chrono::system_clock::time_point & time );

/// \brief Get the epoch time as nanoseconds since the epoch.
NsecTime nsecNow();

/// \brief Convert the time (in integer nanoseconds) to decimal seconds.
double nsecToSec( const NsecTime & time );

/// \brief Convert the time (in seconds) to integer nanoseconds
NsecTime secToNsec( const double & time );

/// \brief return a magic number representing an invalid timestamp
constexpr NsecTime getInvalidTime();

/// \brief Is the time valid? This uses a magic number
///        std::numeric_limits<NsecTime>::min() to represent an invalid time
bool isValid(const NsecTime & time);

} // namespace timing
} // namespace okvis

#endif /* INCLUDE_OKVIS_TIMING_NSECTIMEUTILITIES_HPP_ */
