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
 *  Created on: Mar 27, 2015
 *      Author: Andreas Forster (an.forster@gmail.com)
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Frontend.cpp
 * @brief Source file for the Frontend class.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#include <okvis/Frontend.hpp>

#include <brisk/brisk.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

#include <okvis/ceres/ImuError.hpp>
#include <okvis/VioKeyframeWindowMatchingAlgorithm.hpp>
#include <okvis/IdProvider.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

// Kneip RANSAC
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor.
Frontend::Frontend(size_t numCameras)
    : isInitialized_(false),
      numCameras_(numCameras),
      briskDetectionOctaves_(0),
      briskDetectionThreshold_(50.0),
      briskDetectionAbsoluteThreshold_(800.0),
      briskDetectionMaximumKeypoints_(450),
      briskDescriptionRotationInvariance_(true),
      briskDescriptionScaleInvariance_(false),
      briskMatchingThreshold_(60.0),
      matcher_(
          std::unique_ptr<okvis::DenseMatcher>(new okvis::DenseMatcher(4))),
      keyframeInsertionOverlapThreshold_(0.6),
      keyframeInsertionMatchingRatioThreshold_(0.2) {
  // create mutexes for feature detectors and descriptor extractors
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectorMutexes_.push_back(
        std::unique_ptr<std::mutex>(new std::mutex()));
  }
  initialiseBriskFeatureDetectors();
}

// Detection and descriptor extraction on a per image basis.
bool Frontend::detectAndDescribe(size_t cameraIndex,
                                 std::shared_ptr<okvis::MultiFrame> frameOut,
                                 const okvis::kinematics::Transformation& T_WC,
                                 const std::vector<cv::KeyPoint> * keypoints) {
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_, "Camera index exceeds number of cameras.");
  std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);

  // check there are no keypoints here
  OKVIS_ASSERT_TRUE(Exception, keypoints == nullptr, "external keypoints currently not supported")

  frameOut->setDetector(cameraIndex, featureDetectors_[cameraIndex]);
  frameOut->setExtractor(cameraIndex, descriptorExtractors_[cameraIndex]);

  frameOut->detect(cameraIndex);

  // ExtractionDirection == gravity direction in camera frame
  Eigen::Vector3d g_in_W(0, 0, -1);
  Eigen::Vector3d extractionDirection = T_WC.inverse().C() * g_in_W;
  frameOut->describe(cameraIndex, extractionDirection);

  // set detector/extractor to nullpointer? TODO
  return true;
}

// Matching as well as initialization of landmarks and state.
bool Frontend::dataAssociationAndInitialization(
    okvis::Estimator& estimator,
    okvis::kinematics::Transformation& /*T_WS_propagated*/, // TODO sleutenegger: why is this not used here?
    const okvis::VioParameters &params,
    const std::shared_ptr<okvis::MapPointVector> /*map*/, // TODO sleutenegger: why is this not used here?
    std::shared_ptr<okvis::MultiFrame> framesInOut,
    bool *asKeyframe) {
  // match new keypoints to existing landmarks/keypoints
  // initialise new landmarks (states)
  // outlier rejection by consistency check
  // RANSAC (2D2D / 3D2D)
  // decide keyframe
  // left-right stereo match & init

  // find distortion type
  okvis::cameras::NCameraSystem::DistortionType distortionType = params.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < params.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == params.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet");
  }
  int num3dMatches = 0;

  // first frame? (did do addStates before, so 1 frame minimum in estimator)
  if (estimator.numFrames() > 1) {

    int requiredMatches = 5;

    double uncertainMatchFraction = 0;
    bool rotationOnly = false;

    // match to last keyframe
    TimerSwitchable matchKeyframesTimer("2.4.1 matchToKeyframes");
    switch (distortionType) {
      case okvis::cameras::NCameraSystem::RadialTangential: {
        num3dMatches = matchToKeyframes<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion> > >(
            estimator, params, framesInOut->id(), rotationOnly, false,
            &uncertainMatchFraction);
        break;
      }
      case okvis::cameras::NCameraSystem::Equidistant: {
        num3dMatches = matchToKeyframes<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion> > >(
            estimator, params, framesInOut->id(), rotationOnly, false,
            &uncertainMatchFraction);
        break;
      }
      case okvis::cameras::NCameraSystem::RadialTangential8: {
        num3dMatches = matchToKeyframes<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion8> > >(
            estimator, params, framesInOut->id(), rotationOnly, false,
            &uncertainMatchFraction);
        break;
      }
      default:
        OKVIS_THROW(Exception, "Unsupported distortion type.")
        break;
    }
    matchKeyframesTimer.stop();
    if (!isInitialized_) {
      if (!rotationOnly) {
        isInitialized_ = true;
        LOG(INFO) << "Initialized!";
      }
    }

    if (num3dMatches <= requiredMatches) {
      LOG(WARNING) << "Tracking failure. Number of 3d2d-matches: " << num3dMatches;
    }

    // keyframe decision, at the moment only landmarks that match with keyframe are initialised
    *asKeyframe = *asKeyframe || doWeNeedANewKeyframe(estimator, framesInOut);

    // match to last frame
    TimerSwitchable matchToLastFrameTimer("2.4.2 matchToLastFrame");
    switch (distortionType) {
      case okvis::cameras::NCameraSystem::RadialTangential: {
        matchToLastFrame<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion> > >(
            estimator, params, framesInOut->id(),
            false);
        break;
      }
      case okvis::cameras::NCameraSystem::Equidistant: {
        matchToLastFrame<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion> > >(
            estimator, params, framesInOut->id(),
            false);
        break;
      }
      case okvis::cameras::NCameraSystem::RadialTangential8: {
        matchToLastFrame<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion8> > >(
            estimator, params, framesInOut->id(),
            false);

        break;
      }
      default:
        OKVIS_THROW(Exception, "Unsupported distortion type.")
        break;
    }
    matchToLastFrameTimer.stop();
  } else
    *asKeyframe = true;  // first frame needs to be keyframe

  // do stereo match to get new landmarks
  TimerSwitchable matchStereoTimer("2.4.3 matchStereo");
  switch (distortionType) {
    case okvis::cameras::NCameraSystem::RadialTangential: {
      matchStereo<
          VioKeyframeWindowMatchingAlgorithm<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion> > >(estimator,
                                                                  framesInOut);
      break;
    }
    case okvis::cameras::NCameraSystem::Equidistant: {
      matchStereo<
          VioKeyframeWindowMatchingAlgorithm<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion> > >(estimator,
                                                             framesInOut);
      break;
    }
    case okvis::cameras::NCameraSystem::RadialTangential8: {
      matchStereo<
          VioKeyframeWindowMatchingAlgorithm<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion8> > >(estimator,
                                                                   framesInOut);
      break;
    }
    default:
      OKVIS_THROW(Exception, "Unsupported distortion type.")
      break;
  }
  matchStereoTimer.stop();

  return true;
}

// Propagates pose, speeds and biases with given IMU measurements.
bool Frontend::propagation(const okvis::ImuMeasurementDeque & imuMeasurements,
                           const okvis::ImuParameters & imuParams,
                           okvis::kinematics::Transformation& T_WS_propagated,
                           okvis::SpeedAndBias & speedAndBiases,
                           const okvis::Time& t_start, const okvis::Time& t_end,
                           Eigen::Matrix<double, 15, 15>* covariance,
                           Eigen::Matrix<double, 15, 15>* jacobian) const {
  if (imuMeasurements.size() < 2) {
    LOG(WARNING)
        << "- Skipping propagation as only one IMU measurement has been given to frontend."
        << " Normal when starting up.";
    return 0;
  }
  int measurements_propagated = okvis::ceres::ImuError::propagation(
      imuMeasurements, imuParams, T_WS_propagated, speedAndBiases, t_start,
      t_end, covariance, jacobian);

  return measurements_propagated > 0;
}

// Decision whether a new frame should be keyframe or not.
bool Frontend::doWeNeedANewKeyframe(
    const okvis::Estimator& estimator,
    std::shared_ptr<okvis::MultiFrame> currentFrame) {

  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return true;
  }

  if (!isInitialized_)
    return false;

  double overlap = 0.0;
  double ratio = 0.0;

  // go through all the frames and try to match the initialized keypoints
  for (size_t im = 0; im < currentFrame->numFrames(); ++im) {

    // get the hull of all keypoints in current frame
    std::vector<cv::Point2f> frameBPoints, frameBHull;
    std::vector<cv::Point2f> frameBMatches, frameBMatchesHull;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > frameBLandmarks;

    const size_t numB = currentFrame->numKeypoints(im);
    frameBPoints.reserve(numB);
    frameBLandmarks.reserve(numB);
    Eigen::Vector2d keypoint;
    for (size_t k = 0; k < numB; ++k) {
      currentFrame->getKeypoint(im, k, keypoint);
      // insert it
      frameBPoints.push_back(cv::Point2f(keypoint[0], keypoint[1]));
      // also remember matches
      if (currentFrame->landmarkId(im, k) != 0) {
        frameBMatches.push_back(cv::Point2f(keypoint[0], keypoint[1]));
      }
    }

    if (frameBPoints.size() < 3)
      continue;
    cv::convexHull(frameBPoints, frameBHull);
    if (frameBMatches.size() < 3)
      continue;
    cv::convexHull(frameBMatches, frameBMatchesHull);

    // areas
    double frameBArea = cv::contourArea(frameBHull);
    double frameBMatchesArea = cv::contourArea(frameBMatchesHull);

    // overlap area
    double overlapArea = frameBMatchesArea / frameBArea;
    // matching ratio inside overlap area: count
    int pointsInFrameBMatchesArea = 0;
    if (frameBMatchesHull.size() > 2) {
      for (size_t k = 0; k < frameBPoints.size(); ++k) {
        if (cv::pointPolygonTest(frameBMatchesHull, frameBPoints[k], false)
            > 0) {
          pointsInFrameBMatchesArea++;
        }
      }
    }
    double matchingRatio = double(frameBMatches.size())
        / double(pointsInFrameBMatchesArea);

    // calculate overlap score
    overlap = std::max(overlapArea, overlap);
    ratio = std::max(matchingRatio, ratio);
  }

  // take a decision
  if (overlap > keyframeInsertionOverlapThreshold_
      && ratio > keyframeInsertionMatchingRatioThreshold_)
    return false;
  else
    return true;
}

// Match a new multiframe to existing keyframes
template<class MATCHING_ALGORITHM>
int Frontend::matchToKeyframes(okvis::Estimator& estimator,
                               const okvis::VioParameters & params,
                               const uint64_t currentFrameId,
                               bool& rotationOnly,
                               bool usePoseUncertainty,
                               double* uncertainMatchFraction,
                               bool removeOutliers) {
  rotationOnly = true;
  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return 0;
  }

  int retCtr = 0;
  int numUncertainMatches = 0;

  // go through all the frames and try to match the initialized keypoints
  size_t kfcounter = 0;
  for (size_t age = 1; age < estimator.numFrames(); ++age) {
    uint64_t olderFrameId = estimator.frameIdByAge(age);
    if (!estimator.isKeyframe(olderFrameId))
      continue;
    for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
      MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                           MATCHING_ALGORITHM::Match3D2D,
                                           briskMatchingThreshold_,
                                           usePoseUncertainty);
      matchingAlgorithm.setFrames(olderFrameId, currentFrameId, im, im);

      // match 3D-2D
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
      retCtr += matchingAlgorithm.numMatches();
      numUncertainMatches += matchingAlgorithm.numUncertainMatches();

    }
    kfcounter++;
    if (kfcounter > 2)
      break;
  }

  kfcounter = 0;
  bool firstFrame = true;
  for (size_t age = 1; age < estimator.numFrames(); ++age) {
    uint64_t olderFrameId = estimator.frameIdByAge(age);
    if (!estimator.isKeyframe(olderFrameId))
      continue;
    for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
      MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                           MATCHING_ALGORITHM::Match2D2D,
                                           briskMatchingThreshold_,
                                           usePoseUncertainty);
      matchingAlgorithm.setFrames(olderFrameId, currentFrameId, im, im);

      // match 2D-2D for initialization of new (mono-)correspondences
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
      retCtr += matchingAlgorithm.numMatches();
      numUncertainMatches += matchingAlgorithm.numUncertainMatches();
    }

    // remove outliers
    // only do RANSAC 3D2D with most recent KF
    if (kfcounter == 0 && isInitialized_)
      runRansac3d2d(estimator, params.nCameraSystem,
                    estimator.multiFrame(currentFrameId), removeOutliers);

    bool rotationOnly_tmp = false;
    // do RANSAC 2D2D for initialization only
    if (!isInitialized_) {
      runRansac2d2d(estimator, params, currentFrameId, olderFrameId, true,
                    removeOutliers, rotationOnly_tmp);
    }
    if (firstFrame) {
      rotationOnly = rotationOnly_tmp;
      firstFrame = false;
    }

    kfcounter++;
    if (kfcounter > 1)
      break;
  }

  // calculate fraction of safe matches
  if (uncertainMatchFraction) {
    *uncertainMatchFraction = double(numUncertainMatches) / double(retCtr);
  }

  return retCtr;
}

// Match a new multiframe to the last frame.
template<class MATCHING_ALGORITHM>
int Frontend::matchToLastFrame(okvis::Estimator& estimator,
                               const okvis::VioParameters& params,
                               const uint64_t currentFrameId,
                               bool usePoseUncertainty,
                               bool removeOutliers) {

  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return 0;
  }

  uint64_t lastFrameId = estimator.frameIdByAge(1);

  if (estimator.isKeyframe(lastFrameId)) {
    // already done
    return 0;
  }

  int retCtr = 0;

  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                         MATCHING_ALGORITHM::Match3D2D,
                                         briskMatchingThreshold_,
                                         usePoseUncertainty);
    matchingAlgorithm.setFrames(lastFrameId, currentFrameId, im, im);

    // match 3D-2D
    matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    retCtr += matchingAlgorithm.numMatches();
  }

  runRansac3d2d(estimator, params.nCameraSystem,
                estimator.multiFrame(currentFrameId), removeOutliers);

  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                         MATCHING_ALGORITHM::Match2D2D,
                                         briskMatchingThreshold_,
                                         usePoseUncertainty);
    matchingAlgorithm.setFrames(lastFrameId, currentFrameId, im, im);

    // match 2D-2D for initialization of new (mono-)correspondences
    matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    retCtr += matchingAlgorithm.numMatches();
  }

  // remove outliers
  bool rotationOnly = false;
  if (!isInitialized_)
    runRansac2d2d(estimator, params, currentFrameId, lastFrameId, false,
                  removeOutliers, rotationOnly);

  return retCtr;
}

// Match the frames inside the multiframe to each other to initialise new landmarks.
template<class MATCHING_ALGORITHM>
void Frontend::matchStereo(okvis::Estimator& estimator,
                           std::shared_ptr<okvis::MultiFrame> multiFrame) {

  const size_t camNumber = multiFrame->numFrames();
  const uint64_t mfId = multiFrame->id();

  for (size_t im0 = 0; im0 < camNumber; im0++) {
    for (size_t im1 = im0 + 1; im1 < camNumber; im1++) {
      // first, check the possibility for overlap
      // FIXME: implement this in the Multiframe...!!

      // check overlap
      if(!multiFrame->hasOverlap(im0, im1)){
        continue;
      }

      MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                           MATCHING_ALGORITHM::Match2D2D,
                                           briskMatchingThreshold_,
                                           false);  // TODO: make sure this is changed when switching back to uncertainty based matching
      matchingAlgorithm.setFrames(mfId, mfId, im0, im1);  // newest frame

      // match 2D-2D
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);

      // match 3D-2D
      matchingAlgorithm.setMatchingType(MATCHING_ALGORITHM::Match3D2D);
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);

      // match 2D-3D
      matchingAlgorithm.setFrames(mfId, mfId, im1, im0);  // newest frame
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    }
  }

  // TODO: for more than 2 cameras check that there were no duplications!

  // TODO: ensure 1-1 matching.

  // TODO: no RANSAC ?

  for (size_t im = 0; im < camNumber; im++) {
    const size_t ksize = multiFrame->numKeypoints(im);
    for (size_t k = 0; k < ksize; ++k) {
      if (multiFrame->landmarkId(im, k) != 0) {
        continue;  // already identified correspondence
      }
      multiFrame->setLandmarkId(im, k, okvis::IdProvider::instance().newId());
    }
  }
}

// Perform 3D/2D RANSAC.
int Frontend::runRansac3d2d(okvis::Estimator& estimator,
                            const okvis::cameras::NCameraSystem& nCameraSystem,
                            std::shared_ptr<okvis::MultiFrame> currentFrame,
                            bool removeOutliers) {
  if (estimator.numFrames() < 2) {
    // nothing to match against, we are just starting up.
    return 1;
  }

  /////////////////////
  //   KNEIP RANSAC
  /////////////////////
  int numInliers = 0;

  // absolute pose adapter for Kneip toolchain
  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter adapter(estimator,
                                                                nCameraSystem,
                                                                currentFrame);

  size_t numCorrespondences = adapter.getNumberCorrespondences();
  if (numCorrespondences < 5)
    return numCorrespondences;

  // create a RelativePoseSac problem and RANSAC
  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> ransac;
  std::shared_ptr<
      opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> absposeproblem_ptr(
      new opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem(
          adapter,
          opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem::Algorithm::GP3P));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 9;
  ransac.max_iterations_ = 50;
  // initial guess not needed...
  // run the ransac
  ransac.computeModel(0);

  // assign transformation
  numInliers = ransac.inliers_.size();
  if (numInliers >= 10) {

    // kick out outliers:
    std::vector<bool> inliers(numCorrespondences, false);
    for (size_t k = 0; k < ransac.inliers_.size(); ++k) {
      inliers.at(ransac.inliers_.at(k)) = true;
    }

    for (size_t k = 0; k < numCorrespondences; ++k) {
      if (!inliers[k]) {
        // get the landmark id:
        size_t camIdx = adapter.camIndex(k);
        size_t keypointIdx = adapter.keypointIndex(k);
        uint64_t lmId = currentFrame->landmarkId(camIdx, keypointIdx);

        // reset ID:
        currentFrame->setLandmarkId(camIdx, keypointIdx, 0);

        // remove observation
        if (removeOutliers) {
          estimator.removeObservation(lmId, currentFrame->id(), camIdx,
                                      keypointIdx);
        }
      }
    }
  }
  return numInliers;
}

// Perform 2D/2D RANSAC.
int Frontend::runRansac2d2d(okvis::Estimator& estimator,
                            const okvis::VioParameters& params,
                            uint64_t currentFrameId, uint64_t olderFrameId,
                            bool initializePose,
                            bool removeOutliers,
                            bool& rotationOnly) {
  // match 2d2d
  rotationOnly = false;
  const size_t numCameras = params.nCameraSystem.numCameras();

  size_t totalInlierNumber = 0;
  bool rotation_only_success = false;
  bool rel_pose_success = false;

  // run relative RANSAC
  for (size_t im = 0; im < numCameras; ++im) {

    // relative pose adapter for Kneip toolchain
    opengv::relative_pose::FrameRelativeAdapter adapter(estimator,
                                                        params.nCameraSystem,
                                                        olderFrameId, im,
                                                        currentFrameId, im);

    size_t numCorrespondences = adapter.getNumberCorrespondences();

    if (numCorrespondences < 10)
      continue;  // won't generate meaningful results. let's hope the few correspondences we have are all inliers!!

    // try both the rotation-only RANSAC and the relative one:

    // create a RelativePoseSac problem and RANSAC
    typedef opengv::sac_problems::relative_pose::FrameRotationOnlySacProblem FrameRotationOnlySacProblem;
    opengv::sac::Ransac<FrameRotationOnlySacProblem> rotation_only_ransac;
    std::shared_ptr<FrameRotationOnlySacProblem> rotation_only_problem_ptr(
        new FrameRotationOnlySacProblem(adapter));
    rotation_only_ransac.sac_model_ = rotation_only_problem_ptr;
    rotation_only_ransac.threshold_ = 9;
    rotation_only_ransac.max_iterations_ = 50;

    // run the ransac
    rotation_only_ransac.computeModel(0);

    // get quality
    int rotation_only_inliers = rotation_only_ransac.inliers_.size();
    float rotation_only_ratio = float(rotation_only_inliers)
        / float(numCorrespondences);

    // now the rel_pose one:
    typedef opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem FrameRelativePoseSacProblem;
    opengv::sac::Ransac<FrameRelativePoseSacProblem> rel_pose_ransac;
    std::shared_ptr<FrameRelativePoseSacProblem> rel_pose_problem_ptr(
        new FrameRelativePoseSacProblem(
            adapter, FrameRelativePoseSacProblem::STEWENIUS));
    rel_pose_ransac.sac_model_ = rel_pose_problem_ptr;
    rel_pose_ransac.threshold_ = 9;     //(1.0 - cos(0.5/600));
    rel_pose_ransac.max_iterations_ = 50;

    // run the ransac
    rel_pose_ransac.computeModel(0);

    // assess success
    int rel_pose_inliers = rel_pose_ransac.inliers_.size();
    float rel_pose_ratio = float(rel_pose_inliers) / float(numCorrespondences);

    // decide on success and fill inliers
    std::vector<bool> inliers(numCorrespondences, false);
    if (rotation_only_ratio > rel_pose_ratio || rotation_only_ratio > 0.8) {
      if (rotation_only_inliers > 10) {
        rotation_only_success = true;
      }
      rotationOnly = true;
      totalInlierNumber += rotation_only_inliers;
      for (size_t k = 0; k < rotation_only_ransac.inliers_.size(); ++k) {
        inliers.at(rotation_only_ransac.inliers_.at(k)) = true;
      }
    } else {
      if (rel_pose_inliers > 10) {
        rel_pose_success = true;
      }
      totalInlierNumber += rel_pose_inliers;
      for (size_t k = 0; k < rel_pose_ransac.inliers_.size(); ++k) {
        inliers.at(rel_pose_ransac.inliers_.at(k)) = true;
      }
    }

    // failure?
    if (!rotation_only_success && !rel_pose_success) {
      continue;
    }

    // otherwise: kick out outliers!
    std::shared_ptr<okvis::MultiFrame> multiFrame = estimator.multiFrame(
        currentFrameId);

    for (size_t k = 0; k < numCorrespondences; ++k) {
      size_t idxB = adapter.getMatchKeypointIdxB(k);
      if (!inliers[k]) {

        uint64_t lmId = multiFrame->landmarkId(im, k);
        // reset ID:
        multiFrame->setLandmarkId(im, k, 0);
        // remove observation
        if (removeOutliers) {
          if (lmId != 0 && estimator.isLandmarkAdded(lmId)){
            estimator.removeObservation(lmId, currentFrameId, im, idxB);
          }
        }
      }
    }

    // initialize pose if necessary
    if (initializePose && !isInitialized_) {
      if (rel_pose_success)
        LOG(INFO)
            << "Initializing pose from 2D-2D RANSAC";
      else
        LOG(INFO)
            << "Initializing pose from 2D-2D RANSAC: orientation only";

      Eigen::Matrix4d T_C1C2_mat = Eigen::Matrix4d::Identity();

      okvis::kinematics::Transformation T_SCA, T_WSA, T_SC0, T_WS0;
      uint64_t idA = olderFrameId;
      uint64_t id0 = currentFrameId;
      estimator.getCameraSensorStates(idA, im, T_SCA);
      estimator.get_T_WS(idA, T_WSA);
      estimator.getCameraSensorStates(id0, im, T_SC0);
      estimator.get_T_WS(id0, T_WS0);
      if (rel_pose_success) {
        // update pose
        // if the IMU is used, this will be quickly optimized to the correct scale. Hopefully.
        T_C1C2_mat.topLeftCorner<3, 4>() = rel_pose_ransac.model_coefficients_;

        //initialize with projected length according to motion prior.

        okvis::kinematics::Transformation T_C1C2 = T_SCA.inverse()
            * T_WSA.inverse() * T_WS0 * T_SC0;
        T_C1C2_mat.topRightCorner<3, 1>() = T_C1C2_mat.topRightCorner<3, 1>()
            * std::max(
                0.0,
                double(
                    T_C1C2_mat.topRightCorner<3, 1>().transpose()
                        * T_C1C2.r()));
      } else {
        // rotation only assigned...
        T_C1C2_mat.topLeftCorner<3, 3>() = rotation_only_ransac
            .model_coefficients_;
      }

      // set.
      estimator.set_T_WS(
          id0,
          T_WSA * T_SCA * okvis::kinematics::Transformation(T_C1C2_mat)
              * T_SC0.inverse());
    }
  }

  if (rel_pose_success || rotation_only_success)
    return totalInlierNumber;
  else {
    rotationOnly = true;  // hack...
    return -1;
  }

  return 0;
}

// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or at startup.
void Frontend::initialiseBriskFeatureDetectors() {
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->lock();
  }
  featureDetectors_.clear();
  descriptorExtractors_.clear();
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectors_.push_back(
        std::shared_ptr<cv::FeatureDetector>(
#ifdef __ARM_NEON__
            new cv::GridAdaptedFeatureDetector( 
            new cv::FastFeatureDetector(briskDetectionThreshold_),
                briskDetectionMaximumKeypoints_, 7, 4 ))); // from config file, except the 7x4...
#else
            new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
                briskDetectionThreshold_, briskDetectionOctaves_, 
                briskDetectionAbsoluteThreshold_,
                briskDetectionMaximumKeypoints_)));
#endif
    descriptorExtractors_.push_back(
        std::shared_ptr<cv::DescriptorExtractor>(
            new brisk::BriskDescriptorExtractor(
                briskDescriptionRotationInvariance_,
                briskDescriptionScaleInvariance_)));
  }
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->unlock();
  }
}

}  // namespace okvis
