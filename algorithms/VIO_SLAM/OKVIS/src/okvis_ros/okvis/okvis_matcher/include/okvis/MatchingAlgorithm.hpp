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
 *  Created on: 
 *      Author: Simon Lynen
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file MatchingAlgorithm.hpp
 * @brief Header file for the MatchingAlgorithm class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */


#ifndef INCLUDE_OKVIS_MATCHINGALGORITHM_HPP_
#define INCLUDE_OKVIS_MATCHINGALGORITHM_HPP_

#include <vector>
#include <cstddef>
#include <okvis/assert_macros.hpp>
#include <limits>
#include <map>
#include <memory>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * \class MatchingAlgorithm
 * 
 * \brief an interface for 1-1 matching between lists of things.
 *
 * This superclass defines the interface for a matching algorithm. 
 * Users of the DenseMatcher can implement a child class *or*
 * simply reimplement the interface and call the templated function
 * in the dense matcher.
 * 
 */
class MatchingAlgorithm {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  typedef std::shared_ptr<MatchingAlgorithm> Ptr;

  MatchingAlgorithm();
  virtual ~MatchingAlgorithm();

  /// \brief This will be called exactly once for each call to DenseMatcher::match()
  virtual void doSetup() {
  }

  /// \brief What is the size of list A?
  virtual size_t sizeA() const = 0;
  /// \brief What is the size of list B?
  virtual size_t sizeB() const = 0;

  /// \brief Tree data structure for image space restricted matching
  /// mapping from image row to list of features (indices!)
  typedef std::multimap<size_t, size_t> listB_tree_structure_t;

  /// \brief Get begin iterator for elements of listB to be matched against the given element from list A (indexA)
  /// for a given index of listA, get an iterator into the listB multimap to the start of all elements in listB
  /// that should be matched against indexA
  /// note: implement this in your matching algorithm subclass
  virtual listB_tree_structure_t::iterator getListBStartIterator(size_t indexA);
  /// \brief Get end  iterator for elements of listB to be matched against the given element from list A (indexA)
  /// for a given index of listA, get an iterator into the listB multimap to the end of all elements in listB
  /// that should be matched against indexA
  virtual listB_tree_structure_t::iterator getListBEndIterator(size_t indexA);

  /// \brief Distances above this threshold will not be returned as matches.
  virtual float distanceThreshold() const {
    return std::numeric_limits<float>::max();
  }

  /// \brief By which factor does the first best match has to be better than the second best one.
  virtual float distanceRatioThreshold() const {
    return 0;
  }

  /// \brief Should we skip the item in list A? This will be called once for each item in the list
  virtual bool skipA(size_t /* indexA */) const {
    return false;
  }

  /// \brief Should we skip the item in list B? This will be called many times.
  virtual bool skipB(size_t /* indexB */) const {
    return false;
  }

  /// \brief The "distance" between the two points.
  ///        For points that absolutely don't match. Please use float max.
  virtual float distance(size_t indexA, size_t indexB) const = 0;

  /// \brief A function that tells you how many times setBestMatch() will be called.
  virtual void reserveMatches(size_t numMatches) = 0;

  /// \brief At the end of the matching step, this function is called once
  ///        for each pair of matches discovered.
  virtual void setBestMatch(size_t indexA, size_t indexB, double distance) = 0;

  /// \brief What to return if the match failed
  float matchFailed() const {
    return std::numeric_limits<float>::max();
  }

 private:
  listB_tree_structure_t dummy_;

};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_MATCHINGALGORITHM_HPP_ */
