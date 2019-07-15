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
 * @file implementation/DenseMatcher.hpp
 * @brief Header implementation file for the DenseMatcher class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */

#include <map>

/// \brief okvis Main namespace of this package.
namespace okvis {

// This function creates all the matching threads and assigns the best matches afterwards.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::matchBody(
    void (DenseMatcher::*doWorkPtr)(MatchJob&, MATCHING_ALGORITHM_T*),
    MATCHING_ALGORITHM_T& matchingAlgorithm) {
  // create lock list
  std::mutex* locks = new std::mutex[matchingAlgorithm.sizeB()];

  //the pairing list
  pairing_list_t vpairs;
  // a list with best matches for each "A" point
  std::vector<std::vector<pairing_t> > vMyBest;

  vMyBest.resize(matchingAlgorithm.sizeA());

  // this point is not paired so far, score max
  vpairs.resize(matchingAlgorithm.sizeB(),
                pairing_t(-1, std::numeric_limits<distance_t>::max()));

  // prepare the jobs for the threads
  std::vector<MatchJob> jobs(numMatcherThreads_);
  for (int i = 0; i < numMatcherThreads_; ++i) {
    jobs[i].iThreadID = i;
    jobs[i].vpairs = &vpairs;
    jobs[i].vMyBest = &vMyBest;
    jobs[i].mutexes = locks;
  }

  //create all threads
  //  boost::thread_group matchers;
  for (int i = 0; i < numMatcherThreads_; ++i) {
    matcherThreadPool_->enqueue(doWorkPtr, this, jobs[i], &matchingAlgorithm);
    //    matchers.create_thread(boost::bind(doWorkPtr, this, jobs[i], &matchingAlgorithm));
  }

  //  matchers.join_all();
  matcherThreadPool_->waitForEmptyQueue();

  // Looks like running this in one thread is faster than creating 30+ new threads for every image.
  //TODO(gohlp): distribute this to n threads.

  //  for (int i = 0; i < _numMatcherThreads; ++i)
  //  {
  //	  (this->*doWorkPtr)(jobs[i], &matchingAlgorithm);
  //  }

  matchingAlgorithm.reserveMatches(vpairs.size());

  // assemble the pairs and return
  const distance_t& const_distratiothres = matchingAlgorithm.distanceRatioThreshold();
  const distance_t& const_distthres = matchingAlgorithm.distanceThreshold();
  for (size_t i = 0; i < vpairs.size(); ++i) {
    if (useDistanceRatioThreshold_ && vpairs[i].distance < const_distthres) {
      const std::vector<pairing_t>& best_matches_list =
          vMyBest[vpairs[i].indexA];
      OKVIS_ASSERT_TRUE_DBG(Exception, best_matches_list[0].indexA != -1,
                            "assertion failed");

      if (best_matches_list[1].indexA != -1) {
        const distance_t& best_match_distance = best_matches_list[0].distance;
        const distance_t& second_best_match_distance = best_matches_list[1]
            .distance;
        // Only assign if the distance ratio better than the threshold.
        if (best_match_distance == 0
            || second_best_match_distance / best_match_distance
                > const_distratiothres) {
          matchingAlgorithm.setBestMatch(vpairs[i].indexA, i,
                                         vpairs[i].distance);
        }
      } else {
        // If there is only one matching feature, we assign it.
        matchingAlgorithm.setBestMatch(vpairs[i].indexA, i, vpairs[i].distance);
      }
    } else if (vpairs[i].distance < const_distthres) {
      matchingAlgorithm.setBestMatch(vpairs[i].indexA, i, vpairs[i].distance);
    }
  }

  delete[] locks;
}

// Execute a matching algorithm. This is the fast, templated version. Use this.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::match(MATCHING_ALGORITHM_T & matchingAlgorithm) {
  typedef MATCHING_ALGORITHM_T matching_algorithm_t;
  matchingAlgorithm.doSetup();

  // call the matching body with the linear matching function pointer
  matchBody(&DenseMatcher::template doWorkLinearMatching<matching_algorithm_t>,
            matchingAlgorithm);
}

// Execute a matching algorithm implementing image space matching.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::matchInImageSpace(MATCHING_ALGORITHM_T & matchingAlgorithm) {
  typedef MATCHING_ALGORITHM_T matching_algorithm_t;
  matchingAlgorithm.doSetup();

  // call the matching body with the image space matching function pointer
  matchBody(
      &DenseMatcher::template doWorkImageSpaceMatching<matching_algorithm_t>,
      matchingAlgorithm);
}

// This calculates the distance between to keypoint descriptors. If it is better than the /e numBest_
// found so far, it is included in the aiBest list.
template<typename MATCHING_ALGORITHM_T>
inline void DenseMatcher::listBIteration(
    MATCHING_ALGORITHM_T* matchingAlgorithm, std::vector<pairing_t>& aiBest,
    size_t shortindexA, size_t i) {
  OKVIS_ASSERT_TRUE(std::runtime_error, matchingAlgorithm != NULL,
                    "matching algorithm is NULL");
  typename DenseMatcher::distance_t tmpdist;

  // is this better than worst found so far?
  tmpdist = matchingAlgorithm->distance(shortindexA, i);
  if (tmpdist < aiBest[numBest_ - 1].distance) {
    pairing_t tmp(static_cast<int>(i), tmpdist);
    typename std::vector<pairing_t>::iterator lb = std::lower_bound(
        aiBest.begin(), aiBest.end(), tmp);  //get position for insertion
    typename std::vector<pairing_t>::iterator it, it_next;
    it = it_next = aiBest.end();

    --it;
    --it_next;
    // Insert the new match value into the list
    while (it_next != lb) {
      --it;
      *it_next = *it;  //move value one position to the back
      --it_next;
    }
    *lb = tmp;  //insert both index and score to the correct position to keep strict weak->strong ordering
  }
}

// The threading worker. This matches a keypoint with every other keypoint to find the best match.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::doWorkLinearMatching(
    MatchJob & my_job, MATCHING_ALGORITHM_T * matchingAlgorithm) {
  OKVIS_ASSERT_TRUE(std::runtime_error, matchingAlgorithm != NULL,
                    "matching algorithm is NULL");
  try {
    int start = my_job.iThreadID;
    distance_t const_distthres = matchingAlgorithm->distanceThreshold();
    if (useDistanceRatioThreshold_) {
      // When using the distance ratio threshold, we want to build a list of good matches
      // independent of the threshold first and then later threshold on the ratio.
      const_distthres = std::numeric_limits<distance_t>::max();
    }

    size_t sizeA = matchingAlgorithm->sizeA();
    for (size_t shortindexA = start; shortindexA < sizeA; shortindexA +=
        numMatcherThreads_) {
      if (matchingAlgorithm->skipA(shortindexA))
        continue;

      //typename DenseMatcher::distance_t tmpdist;
      std::vector<pairing_t> & aiBest = (*my_job.vMyBest)[shortindexA];

      // initialize the best match to be -1 (no match) and set the score to be the distance threshold
      // No matches worse than the distance threshold will get through.
      aiBest.resize(numBest_, pairing_t(-1, const_distthres));  //the best x matches for this feature from the long list

      size_t numElementsInListB = matchingAlgorithm->sizeB();
      for (size_t i = 0; i < numElementsInListB; ++i) {
        if (matchingAlgorithm->skipB(i)) {
          continue;
        }

        listBIteration(matchingAlgorithm, aiBest, shortindexA, i);

      }
      assignbest(static_cast<int>(shortindexA), *(my_job.vpairs),
                 *(my_job.vMyBest), my_job.mutexes, 0);  //this call assigns the match and reassigns losing matches recursively
    }
  } catch (const std::exception & e) {
    // \todo Install an error handler in the matching algorithm?
    std::cout << "\033[31mException in matching thread:\033[0m " << e.what();
  }
}

// The threading worker. This matches a keypoint with only a subset of the other keypoints
// to find the best match. (From matchingAlgorithm->getListBStartIterator() to
// MatchingAlgorithm->getListBEndIterator().
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::doWorkImageSpaceMatching(
    MatchJob & my_job, MATCHING_ALGORITHM_T* matchingAlgorithm) {
  OKVIS_ASSERT_TRUE(std::runtime_error, matchingAlgorithm != NULL,
                    "matching algorithm is NULL");
  try {
    int start = my_job.iThreadID;

    size_t numElementsInListB = matchingAlgorithm->sizeB();
    size_t numElementsInListA = matchingAlgorithm->sizeA();

    distance_t const_distthres = matchingAlgorithm->distanceThreshold();
    if (useDistanceRatioThreshold_) {
      // When using the distance ratio threshold, we want to build a list of good matches
      // independent of the threshold first and then later threshold on the ratio.
      const_distthres = std::numeric_limits<distance_t>::max();
    }

    for (size_t shortindexA = start; shortindexA < matchingAlgorithm->sizeA();
        shortindexA += numMatcherThreads_) {
      if (matchingAlgorithm->skipA(shortindexA))
        continue;

      typename DenseMatcher::distance_t tmpdist;
      std::vector<pairing_t>& aiBest = (*my_job.vMyBest)[shortindexA];

      // initialize the best match to be -1 (no match) and set the score to be the distance threshold
      // No matches worse than the distance threshold will get through.
      aiBest.resize(numBest_, pairing_t(-1, const_distthres));  //the best x matches for this feature from the long list

      typename MATCHING_ALGORITHM_T::listB_tree_structure_t::iterator itBegin =
          matchingAlgorithm->getListBStartIterator(shortindexA);
      typename MATCHING_ALGORITHM_T::listB_tree_structure_t::iterator itEnd =
          matchingAlgorithm->getListBEndIterator(shortindexA);
      //check all features from the long list
      for (typename MATCHING_ALGORITHM_T::listB_tree_structure_t::iterator it =
          itBegin; it != itEnd; ++it) {
        size_t i = it->second;

        if (matchingAlgorithm->skipB(i)) {
          continue;
        }

        listBIteration(matchingAlgorithm, aiBest, shortindexA, i);

      }

      assignbest(static_cast<int>(shortindexA), *(my_job.vpairs),
                 *(my_job.vMyBest), my_job.mutexes, 0);  //this call assigns the match and reassigns losing matches recursively
    }

  } catch (const std::exception & e) {
    // \todo Install an error handler in the matching algorithm?
    std::cout << "\033[31mException in matching thread:\033[0m " << e.what();
  }
}

}  // namespace okvis
