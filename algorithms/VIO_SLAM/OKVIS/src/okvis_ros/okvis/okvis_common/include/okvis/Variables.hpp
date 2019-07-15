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
 *  Created on: Apr 22, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Variables.hpp
 * @brief This file contains some typedefs related to state variables.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_VARIABLES_HPP_
#define INCLUDE_OKVIS_VARIABLES_HPP_

#include <Eigen/Core>

/// \brief okvis Main namespace of this package.
namespace okvis {

  typedef Eigen::Matrix<double,9,1> SpeedAndBias;

  typedef Eigen::Matrix<double,3,1> Speed;

  typedef Eigen::Matrix<double,3,1> GyroBias;
  typedef Eigen::Matrix<double,3,1> AccBias;
  typedef Eigen::Matrix<double,6,1> ImuBias;

  typedef Eigen::Matrix<double,3,1> MagnetometerBias;
  typedef Eigen::Matrix<double,1,1> MagnetometerWorldZBias;

  typedef Eigen::Matrix<double,3,1> Wind;
  typedef Eigen::Matrix<double,1,1> Qff;



} // namespace okvis


#endif /* INCLUDE_OKVIS_VARIABLES_HPP_ */
