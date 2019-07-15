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
 * @file DenseMatcher.cpp
 * @brief Source file for the DenseMatcher class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */


#include <okvis/DenseMatcher.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Initialize the dense matcher.
DenseMatcher::DenseMatcher(unsigned char numMatcherThreads,
                           unsigned char numBest,
                           bool useDistanceRatioThreshold)
    : numMatcherThreads_(numMatcherThreads),
      numBest_(numBest),
      useDistanceRatioThreshold_(useDistanceRatioThreshold) {
  matcherThreadPool_.reset(new okvis::ThreadPool(numMatcherThreads_));
}

DenseMatcher::~DenseMatcher() {
  matcherThreadPool_->stop();
}

// Execute a matching algorithm. This is the slow, runtime polymorphic version. Don't use this.
void DenseMatcher::matchSlow(MatchingAlgorithm & matchingAlgorithm)

{
  match(matchingAlgorithm);
}

// A recursive function that reassigns weak matches, if a stronger match is found for a particular point
void DenseMatcher::assignbest(int indexToAssignFromListA,
                              pairing_list_t& vPairsWithScore,
                              std::vector<std::vector<pairing_t> >& aiBestList,
                              std::mutex* mutexes, int startidx) {
  //the top matches for the current index
  const std::vector<pairing_t>& aiBest = aiBestList[indexToAssignFromListA];
  for (int index = startidx; index < numBest_ && aiBest[index].indexA != -1;
      ++index) {
    //fetch index to pair with myidx
    const int pairIndexFromListB = aiBest[index].indexA;
    // synchronize this
    mutexes[pairIndexFromListB].lock();
    if (vPairsWithScore[pairIndexFromListB].indexA == -1) {
      // if we are not paired yet 
      // pair with me
      vPairsWithScore[pairIndexFromListB].indexA = indexToAssignFromListA;
      // set my distance		
      vPairsWithScore[pairIndexFromListB].distance = aiBest[index].distance;
      mutexes[pairIndexFromListB].unlock();
      return;
    } else {
      //already paired, so check the score of that pairing
      if (aiBest[index].distance
          < vPairsWithScore[pairIndexFromListB].distance) {
        // My distance is better!
        // save vals of old pairing
        const int oldPairIndexFromListA = vPairsWithScore[pairIndexFromListB]
            .indexA;
        // pair with me			
        vPairsWithScore[pairIndexFromListB] = pairing_t(indexToAssignFromListA,
                                                        aiBest[index].distance);
        mutexes[pairIndexFromListB].unlock();
        // now reassign the old paring recursivly
        // note: skip element at position zero since we just assigned a match with better score,
        // so we start the reassignment at position 1
        assignbest(oldPairIndexFromListA, vPairsWithScore, aiBestList, mutexes,
                   1);
        return;
      }  //else test next alternative
      mutexes[pairIndexFromListB].unlock();
    }
  }
}

}  // namespace okvis
