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
 *  Created on: Oct 17, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioKeyframeWindowMatchingAlgorithm.hpp
 * @brief Header file for the VioKeyframeWindowMatchingAlgorithm class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_VIOKEYFRAMEWINDOWMATCHINGALGORITHM_HPP_
#define INCLUDE_OKVIS_VIOKEYFRAMEWINDOWMATCHINGALGORITHM_HPP_

#include <memory>

#include <okvis/DenseMatcher.hpp>
#include <okvis/MatchingAlgorithm.hpp>
#include <okvis/Estimator.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/triangulation/ProbabilisticStereoTriangulator.hpp>
#include <okvis/MultiFrame.hpp>
#include <brisk/internal/hamming.h>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * \brief A MatchingAlgorithm implementation
 * \tparam CAMERA_GEOMETRY_T Camera geometry model. See also okvis::cameras::CameraBase.
 */
template<class CAMERA_GEOMETRY_T>
class VioKeyframeWindowMatchingAlgorithm : public okvis::MatchingAlgorithm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  typedef CAMERA_GEOMETRY_T camera_geometry_t;

  enum MatchingTypes {
    Match3D2D = 1,  ///< Match 3D position of established landmarks to 2D keypoint position
    Match2D2D = 2   ///< Match 2D position of established landmarks to 2D keypoint position
  };

  /**
   * @brief Constructor.
   * @param estimator           Estimator.
   * @param matchingType        Matching type. See MatchingTypes enum.
   * @param distanceThreshold   Descriptor distance threshold.
   * @param usePoseUncertainty  Use the pose uncertainty for matching.
   */
  VioKeyframeWindowMatchingAlgorithm(okvis::Estimator& estimator,
                                     int matchingType, float distanceThreshold,
                                     bool usePoseUncertainty = true);

  virtual ~VioKeyframeWindowMatchingAlgorithm();

  /**
   * @brief Set which frames to match.
   * @param mfIdA   The multiframe ID to match against.
   * @param mfIdB   The new multiframe ID.
   * @param camIdA  ID of the frame inside multiframe A to match.
   * @param camIdB  ID of the frame inside multiframe B to match.
   */
  void setFrames(uint64_t mfIdA, uint64_t mfIdB, size_t camIdA, size_t camIdB);

  /**
   * \brief Set the matching type.
   * \see MatchingTypes
   */
  void setMatchingType(int matchingType);

  /// \brief This will be called exactly once for each call to DenseMatcher::match().
  virtual void doSetup();

  /// \brief What is the size of list A?
  virtual size_t sizeA() const;
  /// \brief What is the size of list B?
  virtual size_t sizeB() const;

  /// \brief Get the distance threshold for which matches exceeding it will not be returned as matches.
  virtual float distanceThreshold() const;
  /// \brief Set the distance threshold for which matches exceeding it will not be returned as matches.
  void setDistanceThreshold(float distanceThreshold);

  /// \brief Should we skip the item in list A? This will be called once for each item in the list
  virtual bool skipA(size_t indexA) const {
    return skipA_[indexA];
  }

  /// \brief Should we skip the item in list B? This will be called many times.
  virtual bool skipB(size_t indexB) const {
    return skipB_[indexB];
  }

  /**
   * @brief Calculate the distance between two keypoints.
   * @param indexA Index of the first keypoint.
   * @param indexB Index of the other keypoint.
   * @return Distance between the two keypoint descriptors.
   * @remark Points that absolutely don't match will return float::max.
   */
  virtual float distance(size_t indexA, size_t indexB) const {
    OKVIS_ASSERT_LT_DBG(MatchingAlgorithm::Exception, indexA, sizeA(), "index A out of bounds");
    OKVIS_ASSERT_LT_DBG(MatchingAlgorithm::Exception, indexB, sizeB(), "index B out of bounds");
    const float dist = static_cast<float>(specificDescriptorDistance(
        frameA_->keypointDescriptor(camIdA_, indexA),
        frameB_->keypointDescriptor(camIdB_, indexB)));

    if (dist < distanceThreshold_) {
      if (verifyMatch(indexA, indexB))
        return dist;
    }
    return std::numeric_limits<float>::max();
  }

  /// \brief Geometric verification of a match.
  bool verifyMatch(size_t indexA, size_t indexB) const;

  /// \brief A function that tells you how many times setMatching() will be called.
  /// \warning Currently not implemented to do anything.
  virtual void reserveMatches(size_t numMatches);

  /// \brief At the end of the matching step, this function is called once
  ///        for each pair of matches discovered.
  virtual void setBestMatch(size_t indexA, size_t indexB, double distance);

  /// \brief Get the number of matches.
  size_t numMatches();
  /// \brief Get the number of uncertain matches.
  size_t numUncertainMatches();

  /// \brief access the matching result.
  const okvis::Matches & getMatches() const;

  /// \brief assess the validity of the relative uncertainty computation.
  bool isRelativeUncertaintyValid() {
    return validRelativeUncertainty_;
  }

 private:
  /// \brief This is essentially the map.
  okvis::Estimator* estimator_;

  /// \name Which frames to take
  /// \{
  uint64_t mfIdA_ = 0;
  uint64_t mfIdB_ = 0;
  size_t camIdA_ = 0;
  size_t camIdB_ = 0;

  std::shared_ptr<okvis::MultiFrame> frameA_;
  std::shared_ptr<okvis::MultiFrame> frameB_;
  /// \}

  /// Distances above this threshold will not be returned as matches.
  float distanceThreshold_;

  /// \name Store some transformations that are often used
  /// \{
  /// use a fully relative formulation
  okvis::kinematics::Transformation T_CaCb_;
  okvis::kinematics::Transformation T_CbCa_;
  okvis::kinematics::Transformation T_SaCa_;
  okvis::kinematics::Transformation T_SbCb_;
  okvis::kinematics::Transformation T_WSa_;
  okvis::kinematics::Transformation T_WSb_;
  okvis::kinematics::Transformation T_SaW_;
  okvis::kinematics::Transformation T_SbW_;
  okvis::kinematics::Transformation T_WCa_;
  okvis::kinematics::Transformation T_WCb_;
  okvis::kinematics::Transformation T_CaW_;
  okvis::kinematics::Transformation T_CbW_;
  /// \}

  /// The number of matches.
  size_t numMatches_ = 0;
  /// The number of uncertain matches.
  size_t numUncertainMatches_ = 0;

  /// Focal length of camera used in frame A.
  double fA_ = 0;
  /// Focal length of camera used in frame B.
  double fB_ = 0;

  /// Stored the matching type. See MatchingTypes().
  int matchingType_;

  /// temporarily store all projections
  Eigen::Matrix<double, Eigen::Dynamic, 2> projectionsIntoB_;
  /// temporarily store all projection uncertainties
  Eigen::Matrix<double, Eigen::Dynamic, 2> projectionsIntoBUncertainties_;

  /// Should keypoint[index] in frame A be skipped
  std::vector<bool> skipA_;
  /// Should keypoint[index] in frame B be skipped
  std::vector<bool> skipB_;

  /// Camera center of frame A.
  Eigen::Vector3d pA_W_;
  /// Camera center of frame B.
  Eigen::Vector3d pB_W_;

  /// Temporarily store ray sigmas of frame A.
  std::vector<double> raySigmasA_;
  /// Temporarily store ray sigmas of frame B.
  std::vector<double> raySigmasB_;

  /// Stereo triangulator.
  okvis::triangulation::ProbabilisticStereoTriangulator<camera_geometry_t> probabilisticStereoTriangulator_;

  bool validRelativeUncertainty_ = false;
  bool usePoseUncertainty_ = false;

  /// \brief Calculates the distance between two descriptors.
  // copy from BriskDescriptor.hpp
  uint32_t specificDescriptorDistance(
      const unsigned char * descriptorA,
      const unsigned char * descriptorB) const {
    OKVIS_ASSERT_TRUE_DBG(
        Exception, descriptorA != NULL && descriptorB != NULL,
        "Trying to compare a descriptor with a null description vector");

    return brisk::Hamming::PopcntofXORed(descriptorA, descriptorB, 3/*48 / 16*/);
  }
};

}

#endif /* INCLUDE_OKVIS_VIOKEYFRAMEWINDOWMATCHINGALGORITHM_HPP_ */
