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
 * @file ProbabilisticStereoTriangulator.hpp
 * @brief Header file for the ProbabilisticStereoTriangulator class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_TRIANGULATION_PROBABILISTICSTEREOTRIANGULATOR_HPP_
#define INCLUDE_OKVIS_TRIANGULATION_PROBABILISTICSTEREOTRIANGULATOR_HPP_

#include <memory>

#include <Eigen/Core>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/MultiFrame.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
namespace triangulation {

/**
 * \brief The ProbabilisticStereoTriangulator class
 * \tparam CAMERA_GEOMETRY_T Camera geometry model. See also okvis::cameras::CameraBase.
 */
template<class CAMERA_GEOMETRY_T>
class ProbabilisticStereoTriangulator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * \brief Default constructor; make sure to call resetFrames before triangulation!
   * \param pixelSigma: A (conservative) estimate of the pixel one-sigma uncertainty
   *                    in the keypoint location.
   */
  ProbabilisticStereoTriangulator(double pixelSigma = 0.5);

  /**
   *  \brief Constructor to set frames and relative transformation.
   *  \param frameA_ptr: First multiframe.
   *  \param frameB_ptr: Second multiframe.
   *  \param camIdA: Camera ID for first frame.
   *  \param camIdB: Camera ID for second frame.
   *  \param T_AB: Relative transformation from frameA to frameB.
   *  \param UOplus Oplus-type uncertainty of T_AB.
   *  \param pixelSigma: A (conservative) estimate of the pixel one-sigma uncertainty
   *                     in the keypoint location.
   */
  ProbabilisticStereoTriangulator(std::shared_ptr<okvis::MultiFrame> frameA_ptr,
                                  std::shared_ptr<okvis::MultiFrame> frameB_ptr,
                                  size_t camIdA, size_t camIdB,
                                  const okvis::kinematics::Transformation& T_AB,
                                  const Eigen::Matrix<double, 6, 6>& UOplus,
                                  double pixelSigma = 0.5);

  /**
   *  \brief Reset frames and relative transformation.
   *  \param frameA_ptr: First multiframe.
   *  \param frameB_ptr: Second multiframe.
   *  \param camIdA: Camera ID for first frame.
   *  \param camIdB: Camera ID for second frame.
   *  \param T_AB: Relative transformation from frameA to frameB.
   *  \param UOplus Oplus-type uncertainty of T_AB.
   */
  void resetFrames(std::shared_ptr<okvis::MultiFrame> frameA_ptr,
                   std::shared_ptr<okvis::MultiFrame> frameB_ptr, size_t camIdA,
                   size_t camIdB, const okvis::kinematics::Transformation& T_AB,
                   const Eigen::Matrix<double, 6, 6>& UOplus);

  /// \brief Default destructor.
  ~ProbabilisticStereoTriangulator();

  /**
   *  \brief Triangulation.
   *  \param[in]  keypointIdxA: Index of keypoint to be triangulated in frame A.
   *  \param[in]  keypointIdxB: Index of keypoint to be triangulated in frame B.
   *  \param[out] outHomogeneousPoint_A: Output triangulation in A-coordinates.
   *  \param[out] outCanBeInitializedInaccuarate This will be set to false,
   *              if it can certainly not be initialised (conservative, but inaccurate guess)
   *  \param[in]  sigmaRay Ray uncertainty.
   * \return 3-sigma consistency check result, in A-coordinates.
   */
  bool stereoTriangulate(size_t keypointIdxA, size_t keypointIdxB,
                         Eigen::Vector4d & outHomogeneousPoint_A,
                         bool & outCanBeInitializedInaccuarate,
                         double sigmaRay = -1.0) const;

  /**
   *  \brief Triangulation.
   *  \param[in]  keypointIdxA: Index of keypoint to be triangulated in frame A.
   *  \param[in]  keypointIdxB: Index of keypoint to be triangulated in frame B.
   *  \param[out] outHomogeneousPoint_A: Output triangulation in A-coordinates.
   *  \param[out] outPointUOplus_A: Output uncertainty, represented w.r.t. ceres disturbance, in A-coordinates.
   *  \param[out] outCanBeInitialized Whether or not the triangulation can be considered initialized.
   *  \param[in]  sigmaRay Ray uncertainty.
   *  \return 3-sigma consistency check result.
   */
  bool stereoTriangulate(size_t keypointIdxA, size_t keypointIdxB,
                         Eigen::Vector4d & outHomogeneousPoint_A,
                         Eigen::Matrix3d & outPointUOplus_A,
                         bool & outCanBeInitialized,
                         double sigmaRay = -1.0) const;

  /**
   *  \brief Get triangulation uncertainty.
   *  \param[in] keypointIdxA: Index of keypoint to be triangulated in frame A.
   *  \param[in] keypointIdxB: Index of keypoint to be triangulated in frame B.
   *  \param[in] homogeneousPoint_A: Input triangulated point in A-coordinates.
   *  \param[out] outPointUOplus_A: Output uncertainty, represented w.r.t. ceres disturbance, in A-coordinates.
   *  \param[out] outCanBeInitialized Whether or not the triangulation can be considered initialized.
   */
  void getUncertainty(size_t keypointIdxA, size_t keypointIdxB,
                      const Eigen::Vector4d & homogeneousPoint_A,
                      Eigen::Matrix3d& outPointUOplus_A,
                      bool & outCanBeInitialized) const;

 protected:
  double sigmaRay_;  ///< ray uncertainty
  /// The multiframe A.
  std::shared_ptr<okvis::MultiFrame> frameA_;
  /// The multiframe B.
  std::shared_ptr<okvis::MultiFrame> frameB_;

  /// Camera ID for frame A.
  size_t camIdA_;
  /// Camera ID for frame B.
  size_t camIdB_;

  /// \name Local copy of the relative transformation.
  /// \{
  okvis::kinematics::Transformation T_AB_;
  ::okvis::ceres::PoseParameterBlock poseA_;  // identity
  ::okvis::ceres::PoseParameterBlock poseB_;
  ::okvis::ceres::PoseParameterBlock extrinsics_;
  okvis::kinematics::Transformation T_BA_;
  /// \}

  /// Local copy of relative uncertainty.
  Eigen::Matrix<double, 6, 6> UOplus_;

  /// Information matrix of pose and landmark.
  Eigen::Matrix<double, 9, 9> H_;

  /**
   * @brief Compute the reprojection error.
   * @param[in] frame             Multiframe.
   * @param[in] camId             Camera ID.
   * @param[in] keypointId        ID of keypoint to calculate error for.
   * @param[in] homogeneousPoint  Homogeneous coordinates of point to calculate error for
   * @param[out] outError         Reprojection error.
   * @return True if reprojection was successful.
   */
  bool computeReprojectionError4(
      const std::shared_ptr<okvis::MultiFrame> &frame, size_t camId,
      size_t keypointId, const Eigen::Vector4d& homogeneousPoint,
      double& outError) const;
};

}  // namespace triangulation
}  // namespace okvis

#endif /* INCLUDE_OKVIS_TRIANGULATION_PROBABILISTICSTEREOTRIANGULATOR_HPP_ */
