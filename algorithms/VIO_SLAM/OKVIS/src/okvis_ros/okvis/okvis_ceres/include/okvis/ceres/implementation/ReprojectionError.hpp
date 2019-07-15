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
 *  Created on: Sep 2, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/ReprojectionError.hpp
 * @brief Header implementation file for the ReprojectionError class.
 * @author Stefan Leutenegger
 */

#include <okvis/kinematics/operators.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Default constructor.
template<class GEOMETRY_T>
ReprojectionError<GEOMETRY_T>::ReprojectionError()
    : cameraGeometry_(new camera_geometry_t) {
}

// Construct with measurement and information matrix.
template<class GEOMETRY_T>
ReprojectionError<GEOMETRY_T>::ReprojectionError(
    std::shared_ptr<const camera_geometry_t> cameraGeometry, uint64_t cameraId,
    const measurement_t & measurement, const covariance_t & information) {
  setCameraId(cameraId);
  setMeasurement(measurement);
  setInformation(information);
  setCameraGeometry(cameraGeometry);
}

// Set the information.
template<class GEOMETRY_T>
void ReprojectionError<GEOMETRY_T>::setInformation(
    const covariance_t& information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
}

// This evaluates the error term and additionally computes the Jacobians.
template<class GEOMETRY_T>
bool ReprojectionError<GEOMETRY_T>::Evaluate(double const* const * parameters,
                                             double* residuals,
                                             double** jacobians) const {

  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);  // debug test only
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<class GEOMETRY_T>
bool ReprojectionError<GEOMETRY_T>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobiansMinimal) const {

  // We avoid the use of okvis::kinematics::Transformation here due to quaternion normalization and so forth.
  // This only matters in order to be able to check Jacobians with numeric differentiation chained,
  // first w.r.t. q and then d_alpha.

  // pose: world to sensor transformation
  Eigen::Map<const Eigen::Vector3d> t_WS_W(&parameters[0][0]);
  const Eigen::Quaterniond q_WS(parameters[0][6], parameters[0][3],
                                parameters[0][4], parameters[0][5]);

  // the point in world coordinates
  Eigen::Map<const Eigen::Vector4d> hp_W(&parameters[1][0]);
  //std::cout << hp_W.transpose() << std::endl;

  // the sensor to camera transformation
  Eigen::Map<const Eigen::Vector3d> t_SC_S(&parameters[2][0]);
  const Eigen::Quaterniond q_SC(parameters[2][6], parameters[2][3],
                                parameters[2][4], parameters[2][5]);

  // transform the point into the camera:
  Eigen::Matrix3d C_SC = q_SC.toRotationMatrix();
  Eigen::Matrix3d C_CS = C_SC.transpose();
  Eigen::Matrix4d T_CS = Eigen::Matrix4d::Identity();
  T_CS.topLeftCorner<3, 3>() = C_CS;
  T_CS.topRightCorner<3, 1>() = -C_CS * t_SC_S;
  Eigen::Matrix3d C_WS = q_WS.toRotationMatrix();
  Eigen::Matrix3d C_SW = C_WS.transpose();
  Eigen::Matrix4d T_SW = Eigen::Matrix4d::Identity();
  T_SW.topLeftCorner<3, 3>() = C_SW;
  T_SW.topRightCorner<3, 1>() = -C_SW * t_WS_W;
  Eigen::Vector4d hp_S = T_SW * hp_W;
  Eigen::Vector4d hp_C = T_CS * hp_S;

  // calculate the reprojection error
  measurement_t kp;
  Eigen::Matrix<double, 2, 4> Jh;
  Eigen::Matrix<double, 2, 4> Jh_weighted;
  if (jacobians != NULL) {
    cameraGeometry_->projectHomogeneous(hp_C, &kp, &Jh);
    Jh_weighted = squareRootInformation_ * Jh;
  } else {
    cameraGeometry_->projectHomogeneous(hp_C, &kp);
  }

  measurement_t error = measurement_ - kp;

  // weight:
  measurement_t weighted_error = squareRootInformation_ * error;

  // assign:
  residuals[0] = weighted_error[0];
  residuals[1] = weighted_error[1];

  // check validity:
  bool valid = true;
  if (fabs(hp_C[3]) > 1.0e-8) {
    Eigen::Vector3d p_C = hp_C.template head<3>() / hp_C[3];
    if (p_C[2] < 0.2) {  // 20 cm - not very generic... but reasonable
      //std::cout<<"INVALID POINT"<<std::endl;
      valid = false;
    }
  }

  // calculate jacobians, if required
  // This is pretty close to Paul Furgale's thesis. eq. 3.100 on page 40
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Vector3d p = hp_W.head<3>() - t_WS_W * hp_W[3];
      Eigen::Matrix<double, 4, 6> J;
      J.setZero();
      J.topLeftCorner<3, 3>() = C_SW * hp_W[3];
      J.topRightCorner<3, 3>() = -C_SW * okvis::kinematics::crossMx(p);

      // compute the minimal version
      Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_minimal;
      J0_minimal = Jh_weighted * T_CS * J;
      if (!valid)
        J0_minimal.setZero();

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J0(
          jacobians[0]);
      J0 = J0_minimal * J_lift;

      // if requested, provide minimal Jacobians
      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[0] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > J0_minimal_mapped(
              jacobiansMinimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }

    }
    if (jacobians[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> > J1(
          jacobians[1]);  // map the raw pointer to an Eigen matrix for convenience
      Eigen::Matrix4d T_CW = (T_CS * T_SW);
      J1 = -Jh_weighted * T_CW;
      if (!valid)
        J1.setZero();

      // if requested, provide minimal Jacobians
      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[1] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > J1_minimal_mapped(
              jacobiansMinimal[1]);
          Eigen::Matrix<double, 4, 3> S;
          S.setZero();
          S.topLeftCorner<3, 3>().setIdentity();
          J1_minimal_mapped = J1 * S;  // this is for Euclidean-style perturbation only.
        }
      }
    }
    if (jacobians[2] != NULL) {
      Eigen::Vector3d p = hp_S.head<3>() - t_SC_S * hp_S[3];
      Eigen::Matrix<double, 4, 6> J;
      J.setZero();
      J.topLeftCorner<3, 3>() = C_CS * hp_S[3];
      J.topRightCorner<3, 3>() = -C_CS * okvis::kinematics::crossMx(p);

      // compute the minimal version
      Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J2_minimal;
      J2_minimal = Jh_weighted * J;
      if (!valid)
        J2_minimal.setZero();

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J2(
          jacobians[2]);
      J2 = J2_minimal * J_lift;

      // if requested, provide minimal Jacobians
      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[2] != NULL) {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > J2_minimal_mapped(
              jacobiansMinimal[2]);
          J2_minimal_mapped = J2_minimal;
        }
      }
    }
  }

  return true;
}

}
}
