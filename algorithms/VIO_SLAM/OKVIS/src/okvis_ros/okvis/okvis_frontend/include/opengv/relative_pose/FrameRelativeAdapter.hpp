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
 *  Created on: Mar 7, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file FrameRelativeAdapter.hpp
 * @brief Header file for the FrameRelativeAdapter class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_OPENGV_FRAMERELATIVEADAPTER_HPP_
#define INCLUDE_OKVIS_OPENGV_FRAMERELATIVEADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <okvis/Estimator.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/assert_macros.hpp>

/**
 * \brief Namespace for classes extending the OpenGV library.
 */
namespace opengv {
/**
 * \brief The namespace for the relative pose methods.
 */
namespace relative_pose {

/// \brief Adapter for relative pose RANSAC (2D2D)
class FrameRelativeAdapter : public RelativeAdapterBase {
 private:
  using RelativeAdapterBase::_t12;
  using RelativeAdapterBase::_R12;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor
   * @param estimator     The estimator.
   * @param nCameraSystem Camera configuration and parameters.
   * @param multiFrameIdA The first multiframe.
   * @param camIdA        The camera index for the first multiframe
   *                      in order to access the relevant frame.
   * @param multiFrameIdB The second multiframe.
   * @param camIdB        The camera index for the second multiframe.
   *                      in order to access the relevant frame.
   */
  FrameRelativeAdapter(const okvis::Estimator & estimator,
                       const okvis::cameras::NCameraSystem & nCameraSystem,
                       uint64_t multiFrameIdA, size_t camIdA,
                       uint64_t multiFrameIdB, size_t camIdB);

  virtual ~FrameRelativeAdapter() {
  }

  /// @name Algorithm input
  /// @{

  /**
   * \brief Retrieve the bearing vector of a correspondence in viewpoint 1.
   * \param[in] index The serialized index of the correspondence.
   * \return The corresponding bearing vector.
   */
  virtual opengv::bearingVector_t getBearingVector1(size_t index) const;
  /**
   * \brief Retrieve the bearing vector of a correspondence in viewpoint 2.
   * \param[in] index The serialized index of the correspondence.
   * \return The corresponding bearing vector.
   */
  virtual opengv::bearingVector_t getBearingVector2(size_t index) const;
  /**
   * \brief Retrieve the position of a camera of a correspondence in viewpoint
   *        1 seen from the origin of the viewpoint.
   * \param[in] index The serialized index of the correspondence.
   * \return The position of the corresponding camera seen from the viewpoint
   *         origin.
   */
  virtual opengv::translation_t getCamOffset1(size_t index) const;
  /**
   * \brief Retrieve the rotation from a camera of a correspondence in
   *        viewpoint 1 to the viewpoint origin.
   * \param[in] index The serialized index of the correspondence.
   * \return The rotation from the corresponding camera back to the viewpoint
   *         origin.
   */
  virtual opengv::rotation_t getCamRotation1(size_t index) const;
  /**
   * \brief Retrieve the position of a camera of a correspondence in viewpoint
   *        2 seen from the origin of the viewpoint.
   * \param[in] index The serialized index of the correspondence.
   * \return The position of the corresponding camera seen from the viewpoint
   *         origin.
   */
  virtual opengv::translation_t getCamOffset2(size_t index) const;
  /**
   * \brief Retrieve the rotation from a camera of a correspondence in
   *        viewpoint 2 to the viewpoint origin.
   * \param[in] index The serialized index of the correspondence.
   * \return The rotation from the corresponding camera back to the viewpoint
   *         origin.
   */
  virtual opengv::rotation_t getCamRotation2(size_t index) const;
  /**
   * \brief Retrieve the number of correspondences.
   * \return The number of correspondences.
   */
  virtual size_t getNumberCorrespondences() const;

  /// @}

  // custom:
  /**
   * @brief Obtain the angular standard deviation of the correspondence in frame 1 in [rad].
   * @param index The index of the correspondence.
   * @return The standard deviation in [rad].
   */
  double getSigmaAngle1(size_t index);
  /**
   * @brief Obtain the angular standard deviation of the correspondence in frame 2 in [rad].
   * @param index The index of the correspondence.
   * @return The standard deviation in [rad].
   */
  double getSigmaAngle2(size_t index);
  /**
   * @brief Get the keypoint index in frame 1 of a correspondence.
   * @param index The serialized index of the correspondence.
   * @return The keypoint index of the correspondence in frame 1.
   */
  size_t getMatchKeypointIdxA(size_t index) {
    return matches_.at(index).idxA;
  }
  /**
   * @brief Get the keypoint index in frame 2 of a correspondence.
   * @param index The serialized index of the correspondence.
   * @return The keypoint index of the correspondence in frame 2.
   */
  size_t getMatchKeypointIdxB(size_t index) {
    return matches_.at(index).idxB;
  }
  /**
   * \brief Retrieve the weight of a correspondence. The weight is supposed to
   *        reflect the quality of a correspondence, and typically is between
   *        0 and 1.
   * \warning This is not implemented and always returns 1.0.
   */
  virtual double getWeight(size_t) const {
    return 1.0;
  }  // TODO : figure out, if this is needed

 private:
  /// The bearing vectors of the correspondences in frame 1.
  opengv::bearingVectors_t bearingVectors1_;
  /// The bearing vectors of the correspondences in frame 2.
  opengv::bearingVectors_t bearingVectors2_;
  /// The matching keypoints of both frames.
  okvis::Matches matches_;

  // also store individual uncertainties
  /// The standard deviations of the bearing vectors of frame 1 in [rad].
  std::vector<double> sigmaAngles1_;
  /// The standard deviations of the bearing vectors of frame 2' in [rad].
  std::vector<double> sigmaAngles2_;

};

}
}

#endif /* INCLUDE_OKVIS_OPENGV_FRAMERELATIVEADAPTER_HPP_ */
