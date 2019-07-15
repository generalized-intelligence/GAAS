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
 *  Created on: 2013
 *      Author: Simon Lynen
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file MatchingAlgorithm.cpp
 * @brief Source file for the MatchingAlgorithm class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */


#include <okvis/MatchingAlgorithm.hpp>
#include <okvis/assert_macros.hpp>
#include <exception>

/// \brief okvis Main namespace of this package.
namespace okvis {

MatchingAlgorithm::MatchingAlgorithm() {
}
MatchingAlgorithm::~MatchingAlgorithm() {
}

MatchingAlgorithm::listB_tree_structure_t::iterator MatchingAlgorithm::getListBStartIterator(
    size_t /* indexA */) {
  OKVIS_THROW(
      std::runtime_error,
      "To use the listB iterators, implement this interface in your matching algorithm subclass.");
  return dummy_.end();
}
MatchingAlgorithm::listB_tree_structure_t::iterator MatchingAlgorithm::getListBEndIterator(
    size_t /* indexA */) {
  OKVIS_THROW(
      std::runtime_error,
      "To use the listB iterators, implement this interface in your matching algorithm subclass.");
  return dummy_.end();
}

}  // namespace okvis
