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
 *  Created on: Mar 6, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file FrameAbsolutePoseSacProblem.hpp
 * @brief Header file for the FrameAbsolutePoseSacProblem class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_OPENGV_FRAMEABSOLUTEPOSESACPROBLEM_HPP_
#define INCLUDE_OKVIS_OPENGV_FRAMEABSOLUTEPOSESACPROBLEM_HPP_

#include <opengv/types.hpp>
#include <opengv/absolute_pose/methods.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#pragma GCC diagnostic pop
#include <opengv/absolute_pose/FrameNoncentralAbsoluteAdapter.hpp>
#include <okvis/assert_macros.hpp>

/**
 * \brief Namespace for classes extending the OpenGV library.
 */
namespace opengv {
/**
 * \brief The namespace for the sample consensus problems.
 */
namespace sac_problems {
/**
 * \brief The namespace for the absolute pose methods.
 */
namespace absolute_pose {

/**
 * \brief Provides functions for fitting an absolute-pose model to a set of
 *        bearing-vector to point correspondences, using different algorithms (central
 *        and non-central ones). Used in a sample-consenus paradigm for rejecting
 *        outlier correspondences.
 */
class FrameAbsolutePoseSacProblem : public AbsolutePoseSacProblem {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  typedef AbsolutePoseSacProblem base_t;

  /** The type of adapter that is expected by the methods */
  using base_t::adapter_t;
  /** The possible algorithms for solving this problem */
  using base_t::algorithm_t;
  /** The model we are trying to fit (transformation) */
  using base_t::model_t;

  /**
   * @brief Constructor.
   * @param[in] adapter Visitor holding bearing vectors, world points, etc.
   * @param[in] algorithm The algorithm we want to use.
   * @warning Only okvis::absolute_pose::FrameNoncentralAbsoluteAdapter supported.
   */
  FrameAbsolutePoseSacProblem(adapter_t & adapter, algorithm_t algorithm)
      : base_t(adapter, algorithm),
        adapterDerived_(
            *static_cast<opengv::absolute_pose::FrameNoncentralAbsoluteAdapter*>(&_adapter)) {
    OKVIS_ASSERT_TRUE(
        Exception,
        dynamic_cast<opengv::absolute_pose::FrameNoncentralAbsoluteAdapter*>(&_adapter),
        "only opengv::absolute_pose::FrameNoncentralAbsoluteAdapter supported");
  }

  /**
   * @brief Constructor.
   * @param[in] adapter Visitor holding bearing vectors, world points, etc.
   * @param[in] algorithm The algorithm we want to use.
   * @param[in] indices A vector of indices to be used from all available
   *                    correspondences.
   * @warning Only okvis::absolute_pose::FrameNoncentralAbsoluteAdapter supported.
   */
  FrameAbsolutePoseSacProblem(adapter_t & adapter, algorithm_t algorithm,
                              const std::vector<int> & indices)
      : base_t(adapter, algorithm, indices),
        adapterDerived_(
            *static_cast<opengv::absolute_pose::FrameNoncentralAbsoluteAdapter*>(&_adapter)) {
    OKVIS_ASSERT_TRUE(
        Exception,
        dynamic_cast<opengv::absolute_pose::FrameNoncentralAbsoluteAdapter*>(&_adapter),
        "only opengv::absolute_pose::FrameNoncentralAbsoluteAdapter supported");
  }

  virtual ~FrameAbsolutePoseSacProblem() {
  }

  /**
   * \brief Compute the distances of all samples whith respect to given model
   *        coefficients.
   * \param[in] model The coefficients of the model hypothesis.
   * \param[in] indices The indices of the samples of which we compute distances.
   * \param[out] scores The resulting distances of the selected samples. Low
   *                    distances mean a good fit.
   */
  virtual void getSelectedDistancesToModel(const model_t & model,
                                           const std::vector<int> & indices,
                                           std::vector<double> & scores) const {
    //compute the reprojection error of all points

    //compute inverse transformation
    model_t inverseSolution;
    inverseSolution.block<3, 3>(0, 0) = model.block<3, 3>(0, 0).transpose();
    inverseSolution.col(3) = -inverseSolution.block<3, 3>(0, 0) * model.col(3);

    Eigen::Matrix<double, 4, 1> p_hom;
    p_hom[3] = 1.0;

    for (size_t i = 0; i < indices.size(); i++) {
      //get point in homogeneous form
      p_hom.block<3, 1>(0, 0) = adapterDerived_.getPoint(indices[i]);

      //compute the reprojection (this is working for both central and
      //non-central case)
      point_t bodyReprojection = inverseSolution * p_hom;
      point_t reprojection = adapterDerived_.getCamRotation(indices[i])
          .transpose()
          * (bodyReprojection - adapterDerived_.getCamOffset(indices[i]));
      reprojection = reprojection / reprojection.norm();

      //compute the score
      point_t error = (reprojection
          - adapterDerived_.getBearingVector(indices[i]));
      double error_squared = error.transpose() * error;
      scores.push_back(
          error_squared / adapterDerived_.getSigmaAngle(indices[i]));
    }
  }

 protected:
  /// The adapter holding the bearing, correspondences etc.
  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter & adapterDerived_;

};

}
}
}

#endif /* INCLUDE_OKVIS_OPENGV_FRAMEABSOLUTEPOSESACPROBLEM_HPP_ */
