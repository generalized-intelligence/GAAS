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
 *  Created on: May 22, 2015
 *      Author: Andreas Forster (an.forster@gmail.com)
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file CeresIterationCallback.hpp
 * @brief Header file for the CeresIterationCallback class.
 *        Used to enforce a time limit on the ceres optimization.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_CERESITERATIONCALLBACK_HPP
#define INCLUDE_OKVIS_CERES_CERESITERATIONCALLBACK_HPP

#include <ceres/iteration_callback.h>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/**
 * @brief The CeresIterationCallback class tries to enforce a time limit on the
 *        optimization. It does not guarantee to stay within the time budget as
 *        it assumes the next iteration takes as long as the previous iteration.
 */
class CeresIterationCallback : public ::ceres::IterationCallback {
 public:

  /**
   * @brief The constructor.
   * @param[in] timeLimit Time budget for the optimization.
   * @param[in] iterationMinimum Minimum iterations the optimization should perform
   *            disregarding the time.
   */
  CeresIterationCallback(double timeLimit, int iterationMinimum)
      : timeLimit_(timeLimit),
        iterationMinimum_(iterationMinimum) {
  }

  /// \brief Trivial Destructor.
  ~CeresIterationCallback() {
  }

  /// @brief This method is called after every iteration in ceres.
  /// @param[in] summary The iteration summary.
  ::ceres::CallbackReturnType operator()(
      const ::ceres::IterationSummary& summary) {
    // assume next iteration takes the same time as current iteration
    if (summary.iteration >= iterationMinimum_
        && summary.cumulative_time_in_seconds
            + summary.iteration_time_in_seconds > timeLimit_) {
      return ::ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    }
    return ::ceres::SOLVER_CONTINUE;
  }

  /**
   * @brief setTimeLimit changes time limit of optimization.
   *        If you want to disable the time limit, either set it to a large value,
   *        delete the callback in the ceres options or set the minimum iterations
   *        to the maximum iteration.
   * @param[in] timeLimit desired time limit in seconds
   */
  void setTimeLimit(double timeLimit) {
    timeLimit_ = timeLimit;
  }

  /**
   * @brief setMinimumIterations changes the minimum iterations the optimization
   *        goes through disregarding the time limit
   * @param iterationMinimum
   */
  void setMinimumIterations(int iterationMinimum) {
    iterationMinimum_ = iterationMinimum;
  }

 private:
  double timeLimit_; ///< The set time limit.
  int iterationMinimum_; ///< The set maximum no. iterations.
};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_CERESITERATIONCALLBACK_HPP */
