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
 *********************************************************************************/

/**
 * @file cameras/CameraBase.hpp
 * @brief Header file for the CameraBase class.
 * @author Stefan Leutenegger
 */


#ifndef INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_
#define INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_

#include <vector>
#include <memory>
#include <stdint.h>
#include <Eigen/Core>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include <okvis/cameras/DistortionBase.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \class CameraBase
/// \brief Base class for all camera models.
class CameraBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \class ProjectionStatus
  /// \brief Indicates what happened when applying any of the project functions.
  enum class ProjectionStatus
  {
    Successful,
    OutsideImage,
    Masked,
    Behind,
    Invalid
  };

  /// \brief default Constructor -- does nothing serious
  inline CameraBase()
      : imageWidth_(0),
        imageHeight_(0),
        id_(0)
  {
  }

  /// \brief Constructor for width, height and Id
  inline CameraBase(int imageWidth, int imageHeight, uint64_t id = 0)
        : imageWidth_(imageWidth),
          imageHeight_(imageHeight),
          id_(id)
    {
    }

  /// \brief Destructor -- does nothing
  inline virtual ~CameraBase()
  {
  }

  //////////////////////////////////////////////////////////////
  /// \name Methods related to masking a certain image area as invalid.
  /// @{

  /// \brief Set the mask. It must be the same size as the image and
  /// comply with OpenCV: 0 == masked, nonzero == valid.
  /// Type must be CV_8U1C.
  /// @param[in] mask The actual mask.
  /// @return True if the requirements were followed.
  inline bool setMask(const cv::Mat & mask);

  /// \brief Was a nonzero mask set?
  inline bool hasMask() const;

  /// \brief stop masking
  inline bool removeMask();

  /// \brief Get the mask.
  inline const cv::Mat & mask() const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods related Ids of this camera.
  /// @{

  /// \brief Set an Id.
  inline void setId(uint64_t id)
  {
    id_ = id;
  }

  /// \brief Obtain the Id.
  inline uint64_t id() const
  {
    return id_;
  }

  /// @}

  /// \brief The width of the image in pixels.
  inline uint32_t imageWidth() const
  {
    return imageWidth_;
  }
  /// \brief The height of the image in pixels.
  inline uint32_t imageHeight() const
  {
    return imageHeight_;
  }

  /// \brief obtain all intrinsics
  virtual void getIntrinsics(Eigen::VectorXd & intrinsics) const = 0;

  /// \brief overwrite all intrinsics - use with caution !
  virtual bool setIntrinsics(const Eigen::VectorXd & intrinsics) = 0;

  //////////////////////////////////////////////////////////////
  /// \name Methods to project points
  /// @{

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point      The point in Euclidean coordinates.
  /// @param[out] imagePoint The image point.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus project(const Eigen::Vector3d & point,
                                   Eigen::Vector2d * imagePoint) const = 0;

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Euclidean coordinates.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function w.r.t. the point..
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function w.r.t. the intinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus project(
      const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
      Eigen::Matrix<double, 2, 3> * pointJacobian,
      Eigen::Matrix2Xd * intrinsicsJacobian = NULL) const = 0;

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Euclidean coordinates.
  /// @param[in]  parameters         The intrinsics.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function w.r.t. the point..
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function w.r.t. the intinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus projectWithExternalParameters(
      const Eigen::Vector3d & point, const Eigen::VectorXd & parameters,
      Eigen::Vector2d * imagePoint, Eigen::Matrix<double, 2, 3> * pointJacobian = NULL,
      Eigen::Matrix2Xd * intrinsicsJacobian = NULL) const = 0;

  /// \brief Projects Euclidean points to 2d image points (projection) in a batch.
  ///        Uses projection including distortion models.
  /// @param[in]  points      The points in Euclidean coordinates (one point per column).
  /// @param[out] imagePoints The image points (one point per column).
  /// @param[out] stati       Get information about the success of the projections. See
  ///                         \ref ProjectionStatus for more information.
  virtual void projectBatch(const Eigen::Matrix3Xd & points,
                            Eigen::Matrix2Xd * imagePoints,
                            std::vector<ProjectionStatus> * stati) const = 0;

  /// \brief Projects a point in homogenous coordinates to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point      The point in Homogeneous coordinates.
  /// @param[out] imagePoint The image point.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus projectHomogeneous(
      const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint) const = 0;

  /// \brief Projects a point in homogenous coordinates to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Homogeneous coordinates.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function w.r.t. the point.
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function w.r.t. the intrinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus projectHomogeneous(
      const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint,
      Eigen::Matrix<double, 2, 4> * pointJacobian,
      Eigen::Matrix2Xd * intrinsicsJacobian = NULL) const = 0;

  /// \brief Projects a point in homogenous coordinates to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Homogeneous coordinates.
  /// @param[in]  parameters         The intrinsics.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function w.r.t. the point.
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function w.r.t. the intrinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus projectHomogeneousWithExternalParameters(
      const Eigen::Vector4d & point, const Eigen::VectorXd & parameters,
      Eigen::Vector2d * imagePoint,
      Eigen::Matrix<double, 2, 4> * pointJacobian = NULL,
      Eigen::Matrix2Xd * intrinsicsJacobian = NULL) const = 0;

  /// \brief Projects points in homogenous coordinates to 2d image points (projection) in a batch.
  ///        Uses projection including distortion models.
  /// @param[in]  points      The points in homogeneous coordinates (one point per column).
  /// @param[out] imagePoints The image points (one point per column).
  /// @param[out] stati       Get information about the success of the projections. See
  ///                         \ref ProjectionStatus for more information.
  virtual void projectHomogeneousBatch(
      const Eigen::Matrix4Xd & points, Eigen::Matrix2Xd * imagePoints,
      std::vector<ProjectionStatus> * stati) const = 0;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to backproject points
  /// @{

  /// \brief Back-project a 2d image point into Euclidean space (direction vector).
  /// @param[in]  imagePoint The image point.
  /// @param[out] direction  The Euclidean direction vector.
  /// @return     true on success.
  virtual bool backProject(const Eigen::Vector2d & imagePoint,
                           Eigen::Vector3d * direction) const = 0;

  /// \brief Back-project a 2d image point into Euclidean space (direction vector).
  /// @param[in]  imagePoint         The image point.
  /// @param[out] direction          The Euclidean direction vector.
  /// @param[out] pointJacobian      Jacobian of the back-projection function  w.r.t. the point.
  /// @return     true on success.
  virtual bool backProject(
      const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction,
      Eigen::Matrix<double, 3, 2> * pointJacobian) const = 0;

  /// \brief Back-project 2d image points into Euclidean space (direction vectors).
  /// @param[in]  imagePoints The image points (one point per column).
  /// @param[out] directions  The Euclidean direction vectors (one point per column).
  /// @param[out] success     Success of each of the back-projection
  virtual bool backProjectBatch(const Eigen::Matrix2Xd & imagePoints,
                                Eigen::Matrix3Xd * directions,
                                std::vector<bool> * success) const = 0;

  /// \brief Back-project a 2d image point into homogeneous point (direction vector).
  /// @param[in]  imagePoint The image point.
  /// @param[out] direction  The homogeneous point as direction vector.
  /// @return     true on success.
  virtual bool backProjectHomogeneous(const Eigen::Vector2d & imagePoint,
                                      Eigen::Vector4d * direction) const = 0;

  /// \brief Back-project a 2d image point into homogeneous point (direction vector).
  /// @param[in]  imagePoint         The image point.
  /// @param[out] direction          The homogeneous point as direction vector.
  /// @param[out] pointJacobian      Jacobian of the back-projection function.
  /// @return     true on success.
  virtual bool backProjectHomogeneous(
      const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction,
      Eigen::Matrix<double, 4, 2> * pointJacobian) const = 0;

  /// \brief Back-project 2d image points into homogeneous points (direction vectors).
  /// @param[in]  imagePoints The image points (one point per column).
  /// @param[out] directions  The homogeneous points as direction vectors (one point per column).
  /// @param[out] success     Success of each of the back-projection
  virtual bool backProjectHomogeneousBatch(
      const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix4Xd * directions,
      std::vector<bool> * success) const = 0;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to facilitate unit testing
  /// @{

  /// \brief Creates a random (uniform distribution) image point.
  /// @return A random image point.
  virtual Eigen::Vector2d createRandomImagePoint() const;

  /// \brief Creates a random visible point in Euclidean coordinates.
  /// @param[in] minDist The minimal distance of this point.
  /// @param[in] maxDist The maximum distance of this point.
  /// @return    A random Euclidean point.
  virtual Eigen::Vector3d createRandomVisiblePoint(double minDist = 0.0,
                                                   double maxDist = 10.0) const;

  /// \brief Creates a random visible point in homogeneous coordinates.
  /// @param[in] minDist The minimal distance of this point.
  /// @param[in] maxDist The maximum distance of this point.
  /// @return    A random homogeneous point.
  virtual Eigen::Vector4d createRandomVisibleHomogeneousPoint(double minDist =
                                                                  0.0,
                                                              double maxDist =
                                                                  10.0) const;
  /// @}

  /// \brief Obtain the number of intrinsics parameters.
  virtual int noIntrinsicsParameters() const = 0;

  /// \brief Obtain the type
  virtual std::string type() const = 0;

  /// \brief Obtain the projection type
  virtual const std::string distortionType() const = 0;

 protected:

  /// \brief Check if the keypoint is masked.
  inline bool isMasked(const Eigen::Vector2d& imagePoint) const;

  /// \brief Check if the keypoint is in the image.
  inline bool isInImage(const Eigen::Vector2d& imagePoint) const;

  cv::Mat mask_;  ///< The mask -- empty by default

  int imageWidth_;  ///< image width in pixels
  int imageHeight_;  ///< image height in pixels

  uint64_t id_;  ///< an Id

};

}  // namespace cameras
}  // namespace drltools

#include "implementation/CameraBase.hpp"

#endif /* INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_ */
