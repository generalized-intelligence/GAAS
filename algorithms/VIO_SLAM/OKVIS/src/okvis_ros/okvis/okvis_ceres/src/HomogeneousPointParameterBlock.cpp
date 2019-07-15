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
 *  Created on: Aug 30, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file HomogeneousPointParameterBlock.cpp
 * @brief Source file for the HomogeneousPointParameterBlock class.
 * @author Stefan Leutenegger
 */

#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Default constructor (assumes not fixed).
HomogeneousPointParameterBlock::HomogeneousPointParameterBlock()
    : base_t::ParameterBlockSized(),
      initialized_(false) {
  setFixed(false);
}
// Trivial destructor.
HomogeneousPointParameterBlock::~HomogeneousPointParameterBlock() {
}

// Constructor with estimate and time.
HomogeneousPointParameterBlock::HomogeneousPointParameterBlock(
    const Eigen::Vector4d& point, uint64_t id, bool initialized) {
  setEstimate(point);
  setId(id);
  setInitialized(initialized);
  setFixed(false);
}

// Constructor with estimate and time.
HomogeneousPointParameterBlock::HomogeneousPointParameterBlock(
    const Eigen::Vector3d& point, uint64_t id, bool initialized) {
  setEstimate(Eigen::Vector4d(point[0], point[1], point[2], 1.0));
  setId(id);
  setInitialized(initialized);
  setFixed(false);
}

// setters
// Set estimate of this parameter block.
void HomogeneousPointParameterBlock::setEstimate(const Eigen::Vector4d& point) {
  // hack: only do "Euclidean" points for now...
  for (int i = 0; i < base_t::Dimension; ++i)
    parameters_[i] = point[i];
}

// getters
// Get estimate.
Eigen::Vector4d HomogeneousPointParameterBlock::estimate() const {
  return Eigen::Vector4d(
      Eigen::Vector4d(parameters_[0], parameters_[1], parameters_[2],
                      parameters_[3]));
}

}  // namespace ceres
}  // namespace okvis
