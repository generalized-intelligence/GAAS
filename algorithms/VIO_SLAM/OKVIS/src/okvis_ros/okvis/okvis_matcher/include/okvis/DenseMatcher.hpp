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
 * @file okvis/DenseMatcher.hpp
 * @brief Header file for the DenseMatcher class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */
 
#ifndef INCLUDE_OKVIS_DENSEMATCHER_HPP_
#define INCLUDE_OKVIS_DENSEMATCHER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>

#include <okvis/MatchingAlgorithm.hpp>

#include <okvis/assert_macros.hpp>
#include "ThreadPool.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class matches keypoints from two frames in parallel.
 */
class DenseMatcher {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  typedef std::shared_ptr<DenseMatcher> Ptr;
  /** 
   * @brief Initialize the dense matcher.
   * @param numMatcherThreads Number of matcher threads.
   * @param numBest The number of best matches to keep.
   * @param useDistanceRatioThreshold Instead of using an absolute descriptor distance
   *                                  threshold, compare the smallest distance to the second smallest
   *                                  to decide whether to set it as a match.
   */
  DenseMatcher(unsigned char numMatcherThreads = 8, unsigned char numBest = 4,
               bool useDistanceRatioThreshold = false);

  virtual ~DenseMatcher();

  /// \brief Execute a matching algorithm. This is the fast, templated version. Use this.
  /// \tparam MATCHING_ALGORITHM_T The algorithm to use. E.g. a class derived from MatchingAlgorithm
  template<typename MATCHING_ALGORITHM_T>
  void match(MATCHING_ALGORITHM_T & matchingAlgorithm);

  /// \brief Execute a matching algorithm implementing image space matching
  /// (i.e. match landmarks with features in their image space vicinity)
  /// separate matching function for backwards compatability
  /// note: once a uniform implementation for the "old" matcher and the image space matching has been implemented
  /// these two methods (match, matchInImageSpace) may be merged into one
  template<typename MATCHING_ALGORITHM_T>
  void matchInImageSpace(MATCHING_ALGORITHM_T & matchingAlgorithm);

  /// \brief Execute a matching algorithm. This is the slow, runtime polymorphic version. Don't use this.
  void matchSlow(MatchingAlgorithm & matchingAlgorithm);

  typedef float distance_t;

  /// \brief A struct to save an index and distance pair.
  struct Pairing {
    /// \brief Default constructor.
    Pairing()
        : indexA(-1),
          distance(std::numeric_limits<float>::max()) {
    }
    /// \brief Constructor with maximum distance.
    Pairing(int ia)
        : indexA(ia),
          distance(std::numeric_limits<float>::max()) {
    }
    /// \brief Constructor.
    Pairing(int ia, distance_t d)
        : indexA(ia),
          distance(d) {
    }

    /// \brief Compares distances
    bool operator<(const Pairing & rhs) const {
      return distance < rhs.distance;
    }

    int indexA; ///< Index of paired keypoint.
    distance_t distance; ///< Distance to paired keypoint.
  };

  typedef DenseMatcher::Pairing pairing_t;
  typedef std::vector<pairing_t> pairing_list_t;

  /**
   *  \brief A data struct for the worker thread.
   */
  struct MatchJob {

    //convenience, to make clear what is meant
    typedef DenseMatcher::pairing_t pairing_t;
    typedef std::vector<pairing_t> pairing_list_t;

    /// The list of best matches so far.
    std::vector<std::vector<pairing_t> > * vMyBest;

    /// The thread ID of this job.
    int iThreadID;

    /// The list of pairs for this thread.
    DenseMatcher::pairing_list_t * vpairs;

    /// Mutexes for read/write synchronization in assignment of best match.
    std::mutex* mutexes;
  };

  /**
   * @brief This function creates all the matching threads and assigns the best matches afterwards.
   * @tparam MATCHING_ALGORITHM_T The algorithm to use. E.g. a class derived from MatchingAlgorithm
   * @param doWorkPtr The function that the threads are going to run.
   * @param matchingAlgorithm The matching algorithm.
   */
  template<typename MATCHING_ALGORITHM_T>
  void matchBody(
      void (DenseMatcher::*doWorkPtr)(MatchJob&, MATCHING_ALGORITHM_T*),
      MATCHING_ALGORITHM_T& matchingAlgorithm);

  /**
   * @brief The threading worker. This matches a keypoint with every other keypoint to find the best match.
   * @tparam MATCHING_ALGORITHM_T The algorithm to use. E.g. a class derived from MatchingAlgorithm.
   * @param my_job Struct with information about the job.
   * @param matchingAlgorithm The matching algorithm to use.
   */
  template<typename MATCHING_ALGORITHM_T>
  void doWorkLinearMatching(MatchJob & my_job,
                            MATCHING_ALGORITHM_T* matchingAlgorithm);

  /**
   * @brief The threading worker. This matches a keypoint with only a subset of the other keypoints
   *        to find the best match. (From matchingAlgorithm->getListBStartIterator() to
   *        MatchingAlgorithm->getListBEndIterator().
   * @tparam MATCHING_ALGORITHM_T The algorithm to use. E.g. a class derived from MatchingAlgorithm.
   * @warning The MATCHING_ALGORITHM_T class needs an implementatino of getListBStartIterator() and
   *          getListBEndIterator().
   * @param my_job Struct with information about the job.
   * @param matchingAlgorithm The matching algorithm to use.
   */
  template<typename MATCHING_ALGORITHM_T>
  void doWorkImageSpaceMatching(MatchJob & my_job,
                                MATCHING_ALGORITHM_T* matchingAlgorithm);

  /**
   * @brief A recursive function that reassigns weak matches, if a stronger match is found for a particular point
   * @param[in] myIndexScored The keypoint index that was scored with other keypoints.
   * @param[inout] vPairsWithScore The distances to other keypoints that a single thread calculated.
   * @param[inout] aiBestList The best matches so far.
   * @param locks The mutexes.
   * @param startidx Start the assigning at some index. Used for the recursion. Set to 0.
   */
  void assignbest(int myIndexScored, pairing_list_t & vPairsWithScore,
                  std::vector<std::vector<pairing_t> > & aiBestList,
                  std::mutex* locks, int startidx);

  /**
   * @brief This calculates the distance between to keypoint descriptors. If it is better than the /e numBest_
   *        found so far, it is included in the aiBest list.
   * @tparam MATCHING_ALGORITHM_T The algorithm to use. E.g. a class derived from MatchingAlgorithm.
   * @param matchingAlgorithm The matching algorithm to use.
   * @param[inout] aiBest The \e numBest_ pairings found so far.
   * @param[in] shortindexA Keypoint index in frame A.
   * @param[in] i Keypoint index in frame B.
   */
  template<typename MATCHING_ALGORITHM_T>
  inline void listBIteration(MATCHING_ALGORITHM_T* matchingAlgorithm,
                             std::vector<pairing_t>& aiBest, size_t shortindexA,
                             size_t i);

  unsigned char numMatcherThreads_; ///< The set number of threads.
  unsigned char numBest_;           ///< The set number of best pairings to save.
  bool useDistanceRatioThreshold_;  ///< Use ratio of best and second best match instead of absolute threshold.

  std::unique_ptr<okvis::ThreadPool> matcherThreadPool_;  ///< The threads
};

}  // namespace okvis

#include "implementation/DenseMatcher.hpp"

#endif /* INCLUDE_OKVIS_DENSEMATCHER_HPP_ */
