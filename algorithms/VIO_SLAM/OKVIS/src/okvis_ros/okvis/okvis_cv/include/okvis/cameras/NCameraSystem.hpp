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
 * @file cameras/NCameraSystem.hpp
 * @brief Header file for the NCameraSystem class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_NCAMERASYSTEM_HPP_
#define INCLUDE_OKVIS_NCAMERASYSTEM_HPP_

#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/assert_macros.hpp>
#include "okvis/cameras/CameraBase.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \class NCameraSystem
/// \brief A class that assembles multiple cameras into a system of
/// (potentially different) cameras.
class NCameraSystem
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// The enumeration of the currently supported distortion types.
  enum DistortionType  {
    Equidistant = 0, ///< Use with okvis::cameras::EquidistantDistortion.
    RadialTangential = 1, ///< Use with okvis::cameras::RadialTangentialDistortion.
    NoDistortion = 2,
    RadialTangential8 = 3 ///< Use with okvis::cameras::RadialTangentialDistortion.
  };

  /// \brief Default constructor
  inline NCameraSystem();
  /// \brief Construct with vector of extrinsics and geometries
  /// @param[in] T_SC a vector of extrinsics.
  /// @param[in] cameraGeometries a vector of camera geometries (same length as T_SC).
  /// @param[in] distortionTypes a vector of distortion types (same length as T_SC).
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take a while) should be performed.
  inline NCameraSystem(const std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> & T_SC,
                       const std::vector<std::shared_ptr<const cameras::CameraBase>> & cameraGeometries,
                       const std::vector<DistortionType>& distortionTypes,
                       bool computeOverlaps);

  /// \brief Destructor that doesn't do anything really.
  inline virtual ~NCameraSystem();

  /// \brief Reset with vector of extrinsics and geometries
  /// @param[in] T_SC a vector of extrinsics.
  /// @param[in] cameraGeometries a vector of camera geometries (same length as T_SC).
  /// @param[in] distortionTypes a vector of distortion types (same length as T_SC).
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take a while) should be performed.
  inline void reset(const std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> & T_SC,
                    const std::vector<std::shared_ptr<const cameras::CameraBase>> & cameraGeometries,
                    const std::vector<DistortionType>& distortionTypes,
                    bool computeOverlaps);

  /// \brief Append with a single camera.
  /// @param[in] T_SC extrinsics.
  /// @param[in] cameraGeometry Camera geometry.
  /// @param[in] distortionType Distortion type.
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take a while) should be performed.
  inline void addCamera(std::shared_ptr<const okvis::kinematics::Transformation> T_SC,
                        std::shared_ptr<const cameras::CameraBase> cameraGeometry,
                        DistortionType distortionType,
                        bool computeOverlaps = true);

  /// \brief Obtatin the number of cameras currently added.
  /// @return The number of cameras.
  inline size_t numCameras() const;

  /// \brief compute all the overlaps of fields of view. Attention: can be expensive.
  void computeOverlaps();

  /// \brief get the pose of the IMU frame S with respect to the camera cameraIndex
  /// @param[in] cameraIndex The camera index for which the extrinsics should be returned.
  /// @return T_SC, the extrinsics.
  inline std::shared_ptr<const okvis::kinematics::Transformation> T_SC(size_t cameraIndex) const;

  /// \brief get the camera geometry of camera cameraIndex
  /// @param[in] cameraIndex The camera index for which the camera geometry should be returned.
  /// @return The camera geometry.
  inline std::shared_ptr<const cameras::CameraBase> cameraGeometry(size_t cameraIndex) const;

  /// \brief get the distortion type of the camera
  /// @param[in] cameraIndex The camera index for which the distortion type should be returned.
  /// @return The distortion type
  inline DistortionType distortionType(size_t cameraIndex) const;

  /// \brief Get the overlap mask. Sorry for the weird syntax, but remember that
  /// cv::Mat is essentially a shared pointer.
  /// @param[in] cameraIndexSeenBy The camera index for one camera.
  /// @param[in] cameraIndex The camera index for the other camera.
  /// @return The overlap mask image.
  inline const cv::Mat overlap(size_t cameraIndexSeenBy,
                                 size_t cameraIndex) const;

  /// \brief Can the first camera see parts of the FOV of the second camera?
  /// @param[in] cameraIndexSeenBy The camera index for one camera.
  /// @param[in] cameraIndex The camera index for the other camera.
  /// @return True, if there is at least one pixel of overlap.
  inline bool hasOverlap(size_t cameraIndexSeenBy, size_t cameraIndex) const;

 protected:
  /// \brief Use this to check overlapMats_ and overlaps_ have correct sizes
  /// @return True, if valid.
  inline bool overlapComputationValid() const;
  std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> T_SC_;  ///< Mounting transformations from IMU
  std::vector<std::shared_ptr<const cameras::CameraBase>> cameraGeometries_;  ///< Camera geometries
  std::vector<DistortionType> distortionTypes_;
  std::vector<std::vector<cv::Mat>> overlapMats_;  ///< Overlaps between cameras: mats
  std::vector<std::vector<bool>> overlaps_;  ///< Overlaps between cameras: binary
};

}  // namespace cameras
}  // namespace okvis

#include "implementation/NCameraSystem.hpp"

#endif /* INCLUDE_OKVIS_NCAMERASYSTEM_HPP_ */
