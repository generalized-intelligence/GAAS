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
 *  Created on: Jan 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file implementation/PinholeCamera.hpp
 * @brief Header implementation file for the PinholeCamera class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */


// \brief okvis Main namespace of this package.
namespace okvis {
// \brief cameras Namespace for camera-related functionality.
namespace cameras {

template<class DISTORTION_T>
PinholeCamera<DISTORTION_T>::PinholeCamera(int imageWidth,
                                           int imageHeight,
                                           double focalLengthU,
                                           double focalLengthV,
                                           double imageCenterU,
                                           double imageCenterV,
                                           const distortion_t & distortion,
                                           uint64_t id)
    : CameraBase(imageWidth, imageHeight, id),
    distortion_(distortion),
    fu_(focalLengthU),
    fv_(focalLengthV),
    cu_(imageCenterU),
    cv_(imageCenterV)
{
  intrinsics_[0] = fu_;  //< focalLengthU
  intrinsics_[1] = fv_;  //< focalLengthV
  intrinsics_[2] = cu_;  //< imageCenterU
  intrinsics_[3] = cv_;  //< imageCenterV
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
}

// overwrite all intrinsics - use with caution !
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::setIntrinsics(
    const Eigen::VectorXd & intrinsics)
{
  if (intrinsics.cols() != NumIntrinsics) {
    return false;
  }
  intrinsics_ = intrinsics;
  fu_ = intrinsics[0];  //< focalLengthU
  fv_ = intrinsics[1];  //< focalLengthV
  cu_ = intrinsics[2];  //< imageCenterU
  cv_ = intrinsics[3];  //< imageCenterV
  distortion_.setParameters(
      intrinsics.tail<distortion_t::NumDistortionIntrinsics>());
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
  return true;
}

template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::getIntrinsics(Eigen::VectorXd & intrinsics) const
  {
    intrinsics = intrinsics_;
    Eigen::VectorXd distortionIntrinsics;
    if(distortion_t::NumDistortionIntrinsics > 0) {
      distortion_.getParameters(distortionIntrinsics);
      intrinsics.tail<distortion_t::NumDistortionIntrinsics>() = distortionIntrinsics;
    }
  }

//////////////////////////////////////////
// Methods to project points

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint) const
{
  // handle singularity
  if (fabs(point[2]) < 1.0e-12) {
    return CameraBase::ProjectionStatus::Invalid;
  }

  // projection
  Eigen::Vector2d imagePointUndistorted;
  const double rz = 1.0 / point[2];
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  // distortion
  Eigen::Vector2d imagePoint2;
  if (!distortion_.distort(imagePointUndistorted, &imagePoint2)) {
    return CameraBase::ProjectionStatus::Invalid;
  }

  // scale and offset
  (*imagePoint)[0] = fu_ * imagePoint2[0] + cu_;
  (*imagePoint)[1] = fv_ * imagePoint2[1] + cv_;

  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if(point[2]>0.0){
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 3> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const
{
  // handle singularity
  if (fabs(point[2]) < 1.0e-12) {
    return CameraBase::ProjectionStatus::Invalid;
  }

  // projection
  Eigen::Vector2d imagePointUndistorted;
  const double rz = 1.0 / point[2];
  double rz2 = rz * rz;
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  Eigen::Matrix<double, 2, 3> pointJacobianProjection;
  Eigen::Matrix2Xd intrinsicsJacobianProjection;
  Eigen::Matrix2d distortionJacobian;
  Eigen::Matrix2Xd intrinsicsJacobianDistortion;
  Eigen::Vector2d imagePoint2;

  bool distortionSuccess;
  if (intrinsicsJacobian) {
    // get both Jacobians
    intrinsicsJacobian->resize(2, NumIntrinsics);

    distortionSuccess = distortion_.distort(imagePointUndistorted, &imagePoint2,
                                            &distortionJacobian,
                                            &intrinsicsJacobianDistortion);
    // compute the intrinsics Jacobian
    intrinsicsJacobian->template topLeftCorner<2, 2>() =
        imagePoint2.asDiagonal();
    intrinsicsJacobian->template block<2, 2>(0,2) = Eigen::Matrix2d::Identity();

    if (distortion_t::NumDistortionIntrinsics > 0) {
      intrinsicsJacobian
          ->template bottomRightCorner<2, distortion_t::NumDistortionIntrinsics>() =
          Eigen::Vector2d(fu_, fv_).asDiagonal() * intrinsicsJacobianDistortion;  // chain rule
    }
  } else {
    // only get point Jacobian
    distortionSuccess = distortion_.distort(imagePointUndistorted, &imagePoint2,
                                            &distortionJacobian);
  }

  // compute the point Jacobian in any case
  Eigen::Matrix<double, 2, 3> & J = *pointJacobian;
  J(0, 0) = fu_ * distortionJacobian(0, 0) * rz;
  J(0, 1) = fu_ * distortionJacobian(0, 1) * rz;
  J(0, 2) = -fu_
      * (point[0] * distortionJacobian(0, 0)
          + point[1] * distortionJacobian(0, 1)) * rz2;
  J(1, 0) = fv_ * distortionJacobian(1, 0) * rz;
  J(1, 1) = fv_ * distortionJacobian(1, 1) * rz;
  J(1, 2) = -fv_
      * (point[0] * distortionJacobian(1, 0)
          + point[1] * distortionJacobian(1, 1)) * rz2;

  // scale and offset
  (*imagePoint)[0] = fu_ * imagePoint2[0] + cu_;
  (*imagePoint)[1] = fv_ * imagePoint2[1] + cv_;

  if (!distortionSuccess) {
    return CameraBase::ProjectionStatus::Invalid;
  }
  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if(point[2]>0.0){
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectWithExternalParameters(
    const Eigen::Vector3d & point, const Eigen::VectorXd & parameters,
    Eigen::Vector2d * imagePoint, Eigen::Matrix<double, 2, 3> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const
{
  // handle singularity
  if (fabs(point[2]) < 1.0e-12) {
    return CameraBase::ProjectionStatus::Invalid;
  }

  // parse parameters into human readable form
  const double fu = parameters[0];
  const double fv = parameters[1];
  const double cu = parameters[2];
  const double cv = parameters[3];

  Eigen::VectorXd distortionParameters;
  if (distortion_t::NumDistortionIntrinsics > 0) {
    distortionParameters = parameters
        .template tail<distortion_t::NumDistortionIntrinsics>();
  }

  // projection
  Eigen::Vector2d imagePointUndistorted;
  const double rz = 1.0 / point[2];
  double rz2 = rz * rz;
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  Eigen::Matrix<double, 2, 3> pointJacobianProjection;
  Eigen::Matrix2Xd intrinsicsJacobianProjection;
  Eigen::Matrix2d distortionJacobian;
  Eigen::Matrix2Xd intrinsicsJacobianDistortion;
  Eigen::Vector2d imagePoint2;

  bool distortionSuccess;
  if (intrinsicsJacobian) {
    // get both Jacobians
    intrinsicsJacobian->resize(2, NumIntrinsics);

    distortionSuccess = distortion_.distortWithExternalParameters(imagePointUndistorted,
                                            distortionParameters, &imagePoint2,
                                            &distortionJacobian,
                                            &intrinsicsJacobianDistortion);
    // compute the intrinsics Jacobian
    intrinsicsJacobian->template topLeftCorner<2, 2>() =
        imagePoint2.asDiagonal();
    intrinsicsJacobian->template block<2, 2>(0,2) = Eigen::Matrix2d::Identity();

    if (distortion_t::NumDistortionIntrinsics > 0) {
      intrinsicsJacobian
          ->template bottomRightCorner<2, distortion_t::NumDistortionIntrinsics>() =
          Eigen::Vector2d(fu, fv).asDiagonal() * intrinsicsJacobianDistortion;  // chain rule
    }
  } else {
    // only get point Jacobian
    distortionSuccess = distortion_.distortWithExternalParameters(imagePointUndistorted,
                                            distortionParameters, &imagePoint2,
                                            &distortionJacobian);
  }

  // compute the point Jacobian, if requested
  if(pointJacobian) {
    Eigen::Matrix<double, 2, 3> & J = *pointJacobian;
    J(0, 0) = fu * distortionJacobian(0, 0) * rz;
    J(0, 1) = fu * distortionJacobian(0, 1) * rz;
    J(0, 2) = -fu
        * (point[0] * distortionJacobian(0, 0)
            + point[1] * distortionJacobian(0, 1)) * rz2;
    J(1, 0) = fv * distortionJacobian(1, 0) * rz;
    J(1, 1) = fv * distortionJacobian(1, 1) * rz;
    J(1, 2) = -fv
        * (point[0] * distortionJacobian(1, 0)
            + point[1] * distortionJacobian(1, 1)) * rz2;
  }

  // scale and offset
  (*imagePoint)[0] = fu * imagePoint2[0] + cu;
  (*imagePoint)[1] = fv * imagePoint2[1] + cv;

  if (!distortionSuccess) {
    return CameraBase::ProjectionStatus::Invalid;
  }
  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if(point[2]>0.0){
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}

// Projects Euclidean points to 2d image points (projection) in a batch.
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::projectBatch(
    const Eigen::Matrix3Xd & points, Eigen::Matrix2Xd * imagePoints,
    std::vector<CameraBase::ProjectionStatus> * stati) const
{
  const int numPoints = points.cols();
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector3d point = points.col(i);
    Eigen::Vector2d imagePoint;
    CameraBase::ProjectionStatus status = project(point, &imagePoint);
    imagePoints->col(i) = imagePoint;
    if(stati)
      stati->push_back(status);
  }
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneous(
    const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint) const
{
  Eigen::Vector3d head = point.head<3>();
  if (point[3] < 0) {
    return project(-head, imagePoint);
  } else {
    return project(head, imagePoint);
  }
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneous(
    const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 4> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const
{
  Eigen::Vector3d head = point.head<3>();
  Eigen::Matrix<double, 2, 3> pointJacobian3;
  CameraBase::ProjectionStatus status;
  if (point[3] < 0) {
    status = project(-head, imagePoint,
                                                  &pointJacobian3,
                                                  intrinsicsJacobian);
  } else {
    status = project(head, imagePoint,
                                                  &pointJacobian3,
                                                  intrinsicsJacobian);
  }
  pointJacobian->template bottomRightCorner<2, 1>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<2, 3>() = pointJacobian3;
  return status;
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneousWithExternalParameters(
    const Eigen::Vector4d & point, const Eigen::VectorXd & parameters,
    Eigen::Vector2d * imagePoint, Eigen::Matrix<double, 2, 4> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const
{
  Eigen::Vector3d head = point.head<3>();
  Eigen::Matrix<double, 2, 3> pointJacobian3;
  CameraBase::ProjectionStatus status;
  if (point[3] < 0) {
    status = projectWithExternalParameters(-head, parameters, imagePoint,
                                                  &pointJacobian3,
                                                  intrinsicsJacobian);
  } else {
    status = projectWithExternalParameters(head, parameters, imagePoint,
                                                  &pointJacobian3,
                                                  intrinsicsJacobian);
  }
  pointJacobian->template bottomRightCorner<2, 1>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<2, 3>() = pointJacobian3;
  return status;
}

// Projects points in homogenous coordinates to 2d image points (projection) in a batch.
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::projectHomogeneousBatch(
    const Eigen::Matrix4Xd & points, Eigen::Matrix2Xd * imagePoints,
    std::vector<ProjectionStatus> * stati) const
{
  const int numPoints = points.cols();
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector4d point = points.col(i);
    Eigen::Vector2d imagePoint;
    CameraBase::ProjectionStatus status = projectHomogeneous(point, &imagePoint);
    imagePoints->col(i) = imagePoint;
    if(stati)
      stati->push_back(status);
  }
}

//////////////////////////////////////////
// Methods to backproject points

// Back-project a 2d image point into Euclidean space (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction) const
{
  // unscale and center
  Eigen::Vector2d imagePoint2;
  imagePoint2[0] = (imagePoint[0] - cu_) * one_over_fu_;
  imagePoint2[1] = (imagePoint[1] - cv_) * one_over_fv_;

  // undistort
  Eigen::Vector2d undistortedImagePoint;
  bool success = distortion_.undistort(imagePoint2, &undistortedImagePoint);

  // project 1 into z direction
  (*direction)[0] = undistortedImagePoint[0];
  (*direction)[1] = undistortedImagePoint[1];
  (*direction)[2] = 1.0;

  return success;
}

// Back-project a 2d image point into Euclidean space (direction vector).
template<class DISTORTION_T>
inline bool PinholeCamera<DISTORTION_T>::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction,
    Eigen::Matrix<double, 3, 2> * pointJacobian) const
{
  // unscale and center
  Eigen::Vector2d imagePoint2;
  imagePoint2[0] = (imagePoint[0] - cu_) * one_over_fu_;
  imagePoint2[1] = (imagePoint[1] - cv_) * one_over_fv_;

  // undistort
  Eigen::Vector2d undistortedImagePoint;
  Eigen::Matrix2d pointJacobianUndistortion;
  bool success = distortion_.undistort(imagePoint2, &undistortedImagePoint,
                                       &pointJacobianUndistortion);

  // project 1 into z direction
  (*direction)[0] = undistortedImagePoint[0];
  (*direction)[1] = undistortedImagePoint[1];
  (*direction)[2] = 1.0;

  // Jacobian w.r.t. imagePoint
  Eigen::Matrix<double, 3, 2> outProjectJacobian =
      Eigen::Matrix<double, 3, 2>::Zero();
  outProjectJacobian(0, 0) = one_over_fu_;
  outProjectJacobian(1, 1) = one_over_fv_;

  (*pointJacobian) = outProjectJacobian * pointJacobianUndistortion;  // chain rule

  return success;
}

// Back-project 2d image points into Euclidean space (direction vectors).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectBatch(
    const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix3Xd * directions,
    std::vector<bool> * success) const
{
  const int numPoints = imagePoints.cols();
  directions->row(3) = Eigen::VectorXd::Ones(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector2d imagePoint = imagePoints.col(i);
    Eigen::Vector3d point;
    bool suc = backProject(imagePoint, &point);
    if(success)
      success->push_back(suc);
    directions->col(i) = point;
  }
  return true;
}

// Back-project a 2d image point into homogeneous point (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneous(
    const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction) const
{
  Eigen::Vector3d ray;
  bool success = backProject(imagePoint, &ray);
  direction->template head<3>() = ray;
  (*direction)[4] = 1.0;  // arbitrary
  return success;
}

// Back-project a 2d image point into homogeneous point (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneous(
    const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction,
    Eigen::Matrix<double, 4, 2> * pointJacobian) const
{
  Eigen::Vector3d ray;
  Eigen::Matrix<double, 3, 2> pointJacobian3;
  bool success = backProject(imagePoint, &ray, &pointJacobian3);
  direction->template head<3>() = ray;
  (*direction)[4] = 1.0;  // arbitrary
  pointJacobian->template bottomRightCorner<1,2>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<3, 2>() = pointJacobian3;
  return success;
}

// Back-project 2d image points into homogeneous points (direction vectors).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneousBatch(
    const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix4Xd * directions,
    std::vector<bool> * success) const
{
  const int numPoints = imagePoints.cols();
  directions->row(3) = Eigen::VectorXd::Ones(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector2d imagePoint = imagePoints.col(i);
    Eigen::Vector3d point;
    bool suc = backProject(imagePoint, &point);
    if(success)
      success->push_back(suc);
    directions->template block<3, 1>(0, i) = point;
  }
  return true;
}

}  // namespace cameras
}  // namespace okvis
