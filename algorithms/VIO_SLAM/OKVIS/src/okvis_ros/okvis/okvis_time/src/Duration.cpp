/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Modified and adapted by
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Original Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 *  Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 */

/**
 * @file Duration.cpp
 * @brief This file contains osme helper function for the Duration class.
 * @author Willow Garage Inc.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/Duration.hpp>
#include <okvis/implementation/Duration.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec) {
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part > 1000000000L) {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < INT_MIN || sec_part > INT_MAX)
    throw std::runtime_error("Duration is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec) {
  int64_t sec64 = sec;
  int64_t nsec64 = nsec;

  normalizeSecNSecSigned(sec64, nsec64);

  sec = (int32_t) sec64;
  nsec = (int32_t) nsec64;
}

template class DurationBase<Duration> ;
template class DurationBase<WallDuration> ;
}

