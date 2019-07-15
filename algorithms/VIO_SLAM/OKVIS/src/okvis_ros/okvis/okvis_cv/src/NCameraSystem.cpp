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
 *  Created on: Apr 1, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file NCameraSystem.cpp
 * @brief Sourc file for the NCameraSystem.cpp class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include "okvis/cameras/NCameraSystem.hpp"
#include <opencv2/highgui/highgui.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \brief compute all the overlaps of fields of view. Attention: can be expensive.
void NCameraSystem::computeOverlaps()
{
  OKVIS_ASSERT_TRUE_DBG(
      Exception, T_SC_.size() == cameraGeometries_.size(),
      "Number of extrinsics must match number of camera models!");

  overlapMats_.resize(cameraGeometries_.size());
  overlaps_.resize(cameraGeometries_.size());
  for (size_t cameraIndexSeenBy = 0; cameraIndexSeenBy < overlapMats_.size();
      ++cameraIndexSeenBy) {
    overlapMats_[cameraIndexSeenBy].resize(cameraGeometries_.size());
    overlaps_[cameraIndexSeenBy].resize(cameraGeometries_.size());
    for (size_t cameraIndex = 0; cameraIndex < overlapMats_.size();
        ++cameraIndex) {

      std::shared_ptr<const CameraBase> camera = cameraGeometries_[cameraIndex];

      // self-visibility is trivial:
      if (cameraIndex == cameraIndexSeenBy) {
        // sizing the overlap map:
        overlapMats_[cameraIndexSeenBy][cameraIndex] = cv::Mat::ones(
            camera->imageHeight(), camera->imageWidth(), CV_8UC1);
        overlaps_[cameraIndexSeenBy][cameraIndex] = true;
      } else {
        // sizing the overlap map:
        const size_t height = camera->imageHeight();
        const size_t width = camera->imageWidth();
        cv::Mat& overlapMat = overlapMats_[cameraIndexSeenBy][cameraIndex];
        overlapMat = cv::Mat::zeros(height, width, CV_8UC1);
        // go through all the pixels:
        std::shared_ptr<const CameraBase> otherCamera =
            cameraGeometries_[cameraIndexSeenBy];
        const okvis::kinematics::Transformation T_Cother_C =
            T_SC_[cameraIndexSeenBy]->inverse() * (*T_SC_[cameraIndex]);
        bool hasOverlap = false;
        for (size_t u = 0; u < width; ++u) {
          for (size_t v = 0; v < height; ++v) {
            // backproject
            Eigen::Vector3d ray_C;
            camera->backProject(Eigen::Vector2d(double(u), double(v)), &ray_C);
            // project into other camera
            Eigen::Vector3d ray_Cother = T_Cother_C.C() * ray_C;  // points at infinity, i.e. we only do rotation
            Eigen::Vector2d imagePointInOtherCamera;
            CameraBase::ProjectionStatus status = otherCamera->project(
                ray_Cother, &imagePointInOtherCamera);

            // check the result
            if (status == CameraBase::ProjectionStatus::Successful) {

              Eigen::Vector3d verificationRay;
              otherCamera->backProject(imagePointInOtherCamera,&verificationRay);

              // to avoid an artefact of some distortion models, check again
              // note: (this should be fixed in the distortion implementation)
              if(fabs(ray_Cother.normalized().transpose()*verificationRay.normalized()-1.0)<1.0e-10) {
                // fill in the matrix:
                overlapMat.at<uchar>(v,u) = 1;
                // and remember there is some overlap at all.
                if (!hasOverlap) {
                  overlaps_[cameraIndexSeenBy][cameraIndex] = true;
                }
                hasOverlap = true;
              }
            }
          }
        }
      }
      //std::stringstream name;
      //name << (cameraIndexSeenBy)<<"+"<<(cameraIndex);
      //cv::imshow(name.str().c_str(),255*overlapMats_[cameraIndexSeenBy][cameraIndex]);
    }
  }
  //cv::waitKey();
}

}  // namespace cameras
}  // namespace okvis

