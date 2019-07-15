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
 *  Created on: Mar 30, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis/Frame.hpp
 * @brief Header file for the Frame class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_FRAME_HPP_
#define INCLUDE_OKVIS_FRAME_HPP_

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/features2d/features2d.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include <okvis/Time.hpp>
#include <okvis/assert_macros.hpp>
#include "okvis/cameras/CameraBase.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \class Frame
/// \brief A single camera frame equipped with keypoint detector / extractor.
class Frame
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief a default constructor
  inline Frame()
  {
  }

  /// \brief A constructor that uses the image, specified geometry,
  /// detector and extractor
  /// @param[in] image The image.
  /// @param[in] cameraGeometry The camera geometry.
  /// @param[in] detector The detector to be used.
  /// @param[in] extractor The extractor to be used.
  inline Frame(const cv::Mat & image,
        std::shared_ptr<cameras::CameraBase> & cameraGeometry,
        std::shared_ptr<cv::FeatureDetector> & detector,
        std::shared_ptr<cv::DescriptorExtractor> & extractor);

  /// \brief A simple destructor
  inline virtual ~Frame()
  {
  }

  /// \brief Set the frame image;
  /// @param[in] image The image.
  inline void setImage(const cv::Mat & image);

  /// \brief Set the geometry
  /// @param[in] cameraGeometry The camera geometry.
  inline void setGeometry(std::shared_ptr<const cameras::CameraBase> cameraGeometry);

  /// \brief Set the detector
  /// @param[in] detector The detector to be used.
  inline void setDetector(std::shared_ptr<cv::FeatureDetector> detector);

  /// \brief Set the extractor
  /// @param[in] extractor The extractor to be used.
  inline void setExtractor(std::shared_ptr<cv::DescriptorExtractor> extractor);

  /// \brief Obtain the image
  /// \return The image.
  inline const cv::Mat & image() const;

  /// \brief get the base class geometry (will be slow to use)
  /// \return The camera geometry.
  inline std::shared_ptr<const cameras::CameraBase> geometry() const;

  /// \brief Get the specific geometry (will be fast to use)
  /// \tparam GEOMETRY_T The type for the camera geometry requested.
  /// \return The camera geometry.
  template<class GEOMETRY_T>
  inline std::shared_ptr<const GEOMETRY_T> geometryAs() const;

  /// \brief Detect keypoints. This uses virtual function calls.
  ///        That's a negligibly small overhead for many detections.
  /// \return The number of detected points.
  inline int detect();

  /// \brief Describe keypoints. This uses virtual function calls.
  ///        That's a negligibly small overhead for many detections.
  /// \param extractionDirection The extraction direction in camera frame
  /// \return The number of detected points.
  inline int describe(
      const Eigen::Vector3d & extractionDirection = Eigen::Vector3d(0, 0, 1));

  /// \brief Describe keypoints. This uses virtual function calls.
  ///        That's a negligibly small overhead for many detections.
  /// \tparam GEOMETRY_T The type for the camera geometry requested.
  /// \param extractionDirection the extraction direction in camera frame
  /// \return The number of detected points.
  template<class GEOMETRY_T>
  inline int describeAs(
      const Eigen::Vector3d & extractionDirection = Eigen::Vector3d(0, 0, 1));

  /// \brief Access a specific keypoint in OpenCV format
  /// @param[in] keypointIdx The requested keypoint's index.
  /// @param[out] keypoint The requested keypoint.
  /// \return whether or not the operation was successful.
  inline bool getCvKeypoint(size_t keypointIdx, cv::KeyPoint & keypoint) const;

  /// \brief Get a specific keypoint
  /// @param[in] keypointIdx The requested keypoint's index.
  /// @param[out] keypoint The requested keypoint.
  /// \return whether or not the operation was successful.
  inline bool getKeypoint(size_t keypointIdx, Eigen::Vector2d & keypoint) const;

  /// \brief Get the size of a specific keypoint
  /// @param[in] keypointIdx The requested keypoint's index.
  /// @param[out] keypointSize The requested keypoint's size.
  /// \return whether or not the operation was successful.
  inline bool getKeypointSize(size_t keypointIdx, double & keypointSize) const;

  /// \brief Access the descriptor -- CAUTION: high-speed version.
  /// @param[in] keypointIdx The requested keypoint's index.
  /// \return The descriptor data pointer; NULL if out of bounds.
  inline const unsigned char * keypointDescriptor(size_t keypointIdx);

  /// \brief Set the landmark ID
  /// @param[in] keypointIdx The requested keypoint's index.
  /// @param[in] landmarkId The landmark Id.
  /// \return whether or not the operation was successful.
  inline bool setLandmarkId(size_t keypointIdx, uint64_t landmarkId);

  /// \brief Access the landmark ID
  /// @param[in] keypointIdx The requested keypoint's index.
  /// \return The landmark Id.
  inline uint64_t landmarkId(size_t keypointIdx) const;

  /// \brief Provide keypoints externally.
  /// @param[in] keypoints A vector of keyoints.
  /// \return whether or not the operation was successful.
  inline bool resetKeypoints(const std::vector<cv::KeyPoint> & keypoints);

  /// \brief provide descriptors externally
  /// @param[in] descriptors A vector of descriptors.
  /// \return whether or not the operation was successful.
  inline bool resetDescriptors(const cv::Mat & descriptors);

  /// \brief Get the number of keypoints.
  /// \return The number of keypoints.
  inline size_t numKeypoints() const;

 protected:
  cv::Mat image_;  ///< the image as OpenCV's matrix
  std::shared_ptr<const cameras::CameraBase> cameraGeometry_;  ///< the camera geometry
  std::shared_ptr<cv::FeatureDetector> detector_;  ///< the detector
  std::shared_ptr<cv::DescriptorExtractor> extractor_;  ///< the extractor
  std::vector<cv::KeyPoint> keypoints_;  ///< we store keypoints using OpenCV's struct
  cv::Mat descriptors_;  ///< we store the descriptors using OpenCV's matrices
  std::vector<uint64_t> landmarkIds_;  ///< landmark Id, if associated -- 0 otherwise
};

}  // namespace okvis

#include "implementation/Frame.hpp"

#endif /* INCLUDE_OKVIS_FRAME_HPP_ */
