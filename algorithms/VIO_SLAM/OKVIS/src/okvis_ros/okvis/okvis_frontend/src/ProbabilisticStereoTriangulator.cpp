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
 *  Created on: Oct 18, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ProbabilisticStereoTriangulator.cpp
 * @brief Source file for the ProbabilisticStereoTriangulator class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/triangulation/stereo_triangulation.hpp>
#include <okvis/triangulation/ProbabilisticStereoTriangulator.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
namespace triangulation {

// Default constructor; make sure to call resetFrames before triangulation!
template<class CAMERA_GEOMETRY_T>
ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::ProbabilisticStereoTriangulator(
    double pixelSigma)
    : camIdA_(-1),
      camIdB_(-1) {
  // relative transformation - have a local copy
  T_AB_.setIdentity();
  // relative uncertainty - have a local copy
  UOplus_.setZero();

  sigmaRay_ = pixelSigma / 300.0;
}

// Constructor to set frames and relative transformation.
template<class CAMERA_GEOMETRY_T>
ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::ProbabilisticStereoTriangulator(
    std::shared_ptr<okvis::MultiFrame> frameA_ptr,
    std::shared_ptr<okvis::MultiFrame> frameB_ptr, size_t camIdA, size_t camIdB,
    const okvis::kinematics::Transformation& T_AB,
    const Eigen::Matrix<double, 6, 6>& UOplus, double pixelSigma)
    : frameA_(frameA_ptr),
      frameB_(frameB_ptr),
      camIdA_(camIdA),
      camIdB_(camIdB),
      T_AB_(T_AB),
      UOplus_(UOplus) {
  T_BA_ = T_AB_.inverse();
  // also do all backprojections
//	_frameA_ptr->computeAllBackProjections(false);
//	_frameB_ptr->computeAllBackProjections(false);
  // prepare the pose prior, since this will not change.
  ::okvis::ceres::PoseError poseError(T_AB_, UOplus_.inverse());
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J_minimal;  // Jacobian
  Eigen::Matrix<double, 7, 7, Eigen::RowMajor> J;  // Jacobian
  poseA_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  poseB_ = ::okvis::ceres::PoseParameterBlock(T_AB_, 0, okvis::Time(0));
  extrinsics_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  double residuals[6];
  // evaluate to get the jacobian
  double* parameters = poseB_.parameters();
  double* jacobians = J.data();
  double* jacobians_minimal = J_minimal.data();
  poseError.EvaluateWithMinimalJacobians(&parameters, &residuals[0], &jacobians,
                                         &jacobians_minimal);
  // prepare lhs of Gauss-Newton:
  H_.setZero();
  H_.topLeftCorner<6, 6>() = J_minimal.transpose() * J_minimal;

  sigmaRay_ = pixelSigma
      / std::min(
          frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->focalLengthU(),
          frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->focalLengthU());
}

// Reset frames and relative transformation.
template<class CAMERA_GEOMETRY_T>
void ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::resetFrames(
    std::shared_ptr<okvis::MultiFrame> frameA_ptr,
    std::shared_ptr<okvis::MultiFrame> frameB_ptr, size_t camIdA, size_t camIdB,
    const okvis::kinematics::Transformation& T_AB,
    const Eigen::Matrix<double, 6, 6>& UOplus) {
  T_AB_ = T_AB;
  T_BA_ = T_AB_.inverse();

  frameA_ = frameA_ptr;
  frameB_ = frameB_ptr;
  camIdA_ = camIdA;
  camIdB_ = camIdB;

  UOplus_ = UOplus;
  // also do all backprojections
//	_frameA_ptr->computeAllBackProjections(false);
//	_frameB_ptr->computeAllBackProjections(false);
  // prepare the pose prior, since this will not change.
  ::okvis::ceres::PoseError poseError(T_AB_, UOplus_.inverse());
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J_minimal;  // Jacobian
  Eigen::Matrix<double, 7, 7, Eigen::RowMajor> J;  // Jacobian
  poseA_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  poseB_ = ::okvis::ceres::PoseParameterBlock(T_AB_, 0, okvis::Time(0));
  extrinsics_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  double residuals[6];
  // evaluate to get the jacobian
  double* parameters = poseB_.parameters();
  double* jacobians = J.data();
  double* jacobians_minimal = J_minimal.data();
  poseError.EvaluateWithMinimalJacobians(&parameters, &residuals[0], &jacobians,
                                         &jacobians_minimal);
  // prepare lhs of Gauss-Newton:
  H_.setZero();
  H_.topLeftCorner<6, 6>() = J_minimal.transpose() * J_minimal;

  sigmaRay_ = 0.5
      / std::min(
          frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->focalLengthU(),
          frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->focalLengthU());

}

// Default destructor.
template<class CAMERA_GEOMETRY_T>
ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::~ProbabilisticStereoTriangulator() {
}

// Triangulation.
template<class CAMERA_GEOMETRY_T>
bool ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::stereoTriangulate(
    size_t keypointIdxA, size_t keypointIdxB,
    Eigen::Vector4d& outHomogeneousPoint_A,
    bool & outCanBeInitializedInaccuarate,
    double sigmaRay) const {

  OKVIS_ASSERT_TRUE_DBG(Exception,frameA_&&frameB_,"initialize with frames before use!");

  // chose the source of uncertainty
  double sigmaR = sigmaRay;
  if (sigmaR == -1.0)
    sigmaR = sigmaRay_;

  // call triangulation
  bool isValid;
  bool isParallel;
  Eigen::Vector2d keypointCoordinatesA, keypointCoordinatesB;
  Eigen::Vector3d backProjectionDirectionA_inA, backProjectionDirectionB_inA;

  frameA_->getKeypoint(camIdA_, keypointIdxA, keypointCoordinatesA);
  frameB_->getKeypoint(camIdB_, keypointIdxB, keypointCoordinatesB);

  frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->backProject(
      keypointCoordinatesA, &backProjectionDirectionA_inA);
  frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->backProject(
      keypointCoordinatesB, &backProjectionDirectionB_inA);  // direction in frame B
  backProjectionDirectionB_inA = T_AB_.C() * backProjectionDirectionB_inA;

  Eigen::Vector4d hpA = triangulateFast(
      Eigen::Vector3d(0, 0, 0),  // center of A in A coordinates (0,0,0)
      backProjectionDirectionA_inA.normalized(), T_AB_.r(),  // center of B in A coordinates
      backProjectionDirectionB_inA.normalized(), sigmaR, isValid, isParallel);
  outCanBeInitializedInaccuarate = !isParallel;

  if (!isValid) {
    return false;
  }

  // check reprojection:
  double errA, errB;
  isValid = computeReprojectionError4(frameA_, camIdA_, keypointIdxA, hpA,
                                      errA);
  if (!isValid) {
    return false;
  }
  Eigen::Vector4d outHomogeneousPoint_B = T_BA_ * Eigen::Vector4d(hpA);
  if (!computeReprojectionError4(frameB_, camIdB_, keypointIdxB,
                                 outHomogeneousPoint_B, errB)) {
    isValid = false;
    return false;
  }
  if (errA > 4.0 || errB > 4.0) {
    isValid = false;
  }

  // assign output
  outHomogeneousPoint_A = Eigen::Vector4d(hpA);

  return isValid;
}

// Triangulation.
template<class CAMERA_GEOMETRY_T>
bool ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::stereoTriangulate(
    size_t keypointIdxA, size_t keypointIdxB,
    Eigen::Vector4d& outHomogeneousPoint_A, Eigen::Matrix3d& outPointUOplus_A,
    bool& outCanBeInitialized, double sigmaRay) const {
  OKVIS_ASSERT_TRUE_DBG(Exception,frameA_&&frameB_,"initialize with frames before use!");

  // get the triangulation
  bool canBeInitialized;
  if (!stereoTriangulate(keypointIdxA, keypointIdxB, outHomogeneousPoint_A, canBeInitialized,
                         sigmaRay)){
    return false;
  }

  // and get the uncertainty /
  getUncertainty(keypointIdxA, keypointIdxB, outHomogeneousPoint_A,
                 outPointUOplus_A, outCanBeInitialized);
  outCanBeInitialized &= canBeInitialized; // be conservative -- if the initial one failed, the 2nd should, too...
  return true;
}

// Get triangulation uncertainty.
template<class CAMERA_GEOMETRY_T>
void ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::getUncertainty(
    size_t keypointIdxA, size_t keypointIdxB,
    const Eigen::Vector4d& homogeneousPoint_A,
    Eigen::Matrix3d& outPointUOplus_A, bool& outCanBeInitialized) const {
  OKVIS_ASSERT_TRUE_DBG(Exception,frameA_&&frameB_,"initialize with frames before use!");

  // also get the point in the other coordinate representation
  //Eigen::Vector4d& homogeneousPoint_B=_T_BA*homogeneousPoint_A;
  Eigen::Vector4d hPA = homogeneousPoint_A;

  // calculate point uncertainty by constructing the lhs of the Gauss-Newton equation system.
  // note: the transformation T_WA is assumed constant and identity w.l.o.g.
  Eigen::Matrix<double, 9, 9> H = H_;

  //	keypointA_t& kptA = _frameA_ptr->keypoint(keypointIdxA);
  //	keypointB_t& kptB = _frameB_ptr->keypoint(keypointIdxB);
  Eigen::Vector2d kptA, kptB;
  frameA_->getKeypoint(camIdA_, keypointIdxA, kptA);
  frameB_->getKeypoint(camIdB_, keypointIdxB, kptB);

  // assemble the stuff from the reprojection errors
  double keypointStdDev;
  frameA_->getKeypointSize(camIdA_, keypointIdxA, keypointStdDev);
  keypointStdDev = 0.8 * keypointStdDev / 12.0;
  Eigen::Matrix2d inverseMeasurementCovariance = Eigen::Matrix2d::Identity()
      * (1.0 / (keypointStdDev * keypointStdDev));
  ::okvis::ceres::ReprojectionError<CAMERA_GEOMETRY_T> reprojectionErrorA(
      frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_), 0, kptA,
      inverseMeasurementCovariance);
  //typename keypointA_t::measurement_t residualA;
  Eigen::Matrix<double, 2, 1> residualA;
  Eigen::Matrix<double, 2, 4, Eigen::RowMajor> J_hpA;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_hpA_min;
  double* jacobiansA[3];
  jacobiansA[0] = 0;  // do not calculate, T_WA is fixed identity transform
  jacobiansA[1] = J_hpA.data();
  jacobiansA[2] = 0;  // fixed extrinsics
  double* jacobiansA_min[3];
  jacobiansA_min[0] = 0;  // do not calculate, T_WA is fixed identity transform
  jacobiansA_min[1] = J_hpA_min.data();
  jacobiansA_min[2] = 0;  // fixed extrinsics
  const double* parametersA[3];
  //const double* test = _poseA.parameters();
  parametersA[0] = poseA_.parameters();
  parametersA[1] = hPA.data();
  parametersA[2] = extrinsics_.parameters();
  reprojectionErrorA.EvaluateWithMinimalJacobians(parametersA, residualA.data(),
                                                  jacobiansA, jacobiansA_min);

  inverseMeasurementCovariance.setIdentity();
  frameB_->getKeypointSize(camIdB_, keypointIdxB, keypointStdDev);
  keypointStdDev = 0.8 * keypointStdDev / 12.0;
  inverseMeasurementCovariance *= 1.0 / (keypointStdDev * keypointStdDev);

  ::okvis::ceres::ReprojectionError<CAMERA_GEOMETRY_T> reprojectionErrorB(
      frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_), 0, kptB,
      inverseMeasurementCovariance);
  Eigen::Matrix<double, 2, 1> residualB;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_TB;
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J_TB_min;
  Eigen::Matrix<double, 2, 4, Eigen::RowMajor> J_hpB;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_hpB_min;
  double* jacobiansB[3];
  jacobiansB[0] = J_TB.data();
  jacobiansB[1] = J_hpB.data();
  jacobiansB[2] = 0;  // fixed extrinsics
  double* jacobiansB_min[3];
  jacobiansB_min[0] = J_TB_min.data();
  jacobiansB_min[1] = J_hpB_min.data();
  jacobiansB_min[2] = 0;  // fixed extrinsics
  const double* parametersB[3];
  parametersB[0] = poseB_.parameters();
  parametersB[1] = hPA.data();
  parametersB[2] = extrinsics_.parameters();
  reprojectionErrorB.EvaluateWithMinimalJacobians(parametersB, residualB.data(),
                                                  jacobiansB, jacobiansB_min);

  // evaluate again closer:
  hPA.head<3>() = 0.8 * (hPA.head<3>() - T_AB_.r() / 2.0 * hPA[3])
      + T_AB_.r() / 2.0 * hPA[3];
  reprojectionErrorB.EvaluateWithMinimalJacobians(parametersB, residualB.data(),
                                                  jacobiansB, jacobiansB_min);
  if (residualB.transpose() * residualB < 4.0)
    outCanBeInitialized = false;
  else
    outCanBeInitialized = true;

  // now add to H:
  H.bottomRightCorner<3, 3>() += J_hpA_min.transpose() * J_hpA_min;
  H.topLeftCorner<6, 6>() += J_TB_min.transpose() * J_TB_min;
  H.topRightCorner<6, 3>() += J_TB_min.transpose() * J_hpB_min;
  H.bottomLeftCorner<3, 6>() += J_hpB_min.transpose() * J_TB_min;
  H.bottomRightCorner<3, 3>() += J_hpB_min.transpose() * J_hpB_min;

  // invert (if invertible) to get covariance:
  Eigen::Matrix<double, 9, 9> cov;
  if (H.colPivHouseholderQr().rank() < 9) {
    outCanBeInitialized = false;
    return;
  }
  cov = H.inverse();  // FIXME: use the QR decomposition for this...
  outPointUOplus_A = cov.bottomRightCorner<3, 3>();
}

// Compute the reprojection error.
template<class CAMERA_GEOMETRY_T>
bool ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::computeReprojectionError4(
    const std::shared_ptr<okvis::MultiFrame>& frame, size_t camId,
    size_t keypointId, const Eigen::Vector4d& homogeneousPoint,
    double& outError) const {

  OKVIS_ASSERT_LT_DBG(Exception, keypointId, frame->numKeypoints(camId),
      "Index out of bounds");
  Eigen::Vector2d y;
  okvis::cameras::CameraBase::ProjectionStatus status = frame
      ->geometryAs<CAMERA_GEOMETRY_T>(camId)->projectHomogeneous(
      homogeneousPoint, &y);
  if (status == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
    Eigen::Vector2d k;
    Eigen::Matrix2d inverseCov = Eigen::Matrix2d::Identity();
    double keypointStdDev;
    frame->getKeypoint(camId, keypointId, k);
    frame->getKeypointSize(camId, keypointId, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    inverseCov *= 1.0 / (keypointStdDev * keypointStdDev);

    y -= k;
    outError = y.dot(inverseCov * y);
    return true;
  } else
    return false;
}

template class ProbabilisticStereoTriangulator<
    okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion> > ;
template class ProbabilisticStereoTriangulator<
    okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion> > ;
template class ProbabilisticStereoTriangulator<
    okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8> > ;

}  // namespace triangulation
}  // namespace okvis
