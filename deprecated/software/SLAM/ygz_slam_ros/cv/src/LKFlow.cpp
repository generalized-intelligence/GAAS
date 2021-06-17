#include "ygz/LKFlow.h"
#include "ygz/Align.h"
#include "ygz/Feature.h"
#include "ygz/MapPoint.h"

#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace ygz {

    int LKFlow(
            const shared_ptr<Frame> ref,
            const shared_ptr<Frame> current,
            VecVector2f &trackPts,
            bool keepNotConverged
    ) {

        if (ref->mPyramidLeft.empty())
            ref->ComputeImagePyramid();
        if (current->mPyramidLeft.empty())
            current->ComputeImagePyramid();

        if (trackPts.empty()) {
            trackPts.resize(ref->mFeaturesLeft.size());
            for (size_t i = 0; i < ref->mFeaturesLeft.size(); i++)
                if (ref->mFeaturesLeft[i])
                    trackPts[i] = ref->mFeaturesLeft[i]->mPixel;
                else
                    trackPts[i] = Vector2f(-1, -1);
        } else {
            // 已经给定了一些点，则按照这些初始值运行
            assert(ref->mFeaturesLeft.size() == trackPts.size());
        }

        // 匹配局部地图用的 patch, 默认8x8
        uchar patch[align_patch_area] = {0};
        // 带边界的，左右各1个像素
        uchar patch_with_border[(align_patch_size + 2) * (align_patch_size + 2)] = {0};

        int successPts = 0;
        for (size_t i = 0; i < ref->mFeaturesLeft.size(); i++) {
            auto feat = ref->mFeaturesLeft[i];
            if (feat == nullptr) {    // 特征无效
                trackPts[i] = Vector2f(-1, -1);
                continue;
            }
            // from coarse to fine
            Vector2f refPixel = feat->mPixel;
            Vector2f trackedPos = trackPts[i];  // 第零层分辨率下的位置
            bool success = true;

            for (int lvl = setting::numPyramid - 1; lvl >= 0; lvl--) {

                float scale = setting::scaleFactors[lvl];
                float invScale = setting::invScaleFactors[lvl];

                Vector2f posLvl = trackedPos * invScale;   // 第lvl层下的位置
                Vector2f refPixelLvl = refPixel * invScale;
                cv::Mat &img_ref = ref->mPyramidLeft[lvl];

                // copy the patch with boarder
                uchar *patch_ptr = patch_with_border;
                const int ix = floor(refPixelLvl[0]);
                const int iy = floor(refPixelLvl[1]);
                const float xx = refPixelLvl[0] - ix;
                const float yy = refPixelLvl[1] - iy;

                for (int y = 0; y < align_patch_size + 2; y++) {
                    for (int x = 0; x < align_patch_size + 2; x++, ++patch_ptr) {
                        const int dx = x - align_halfpatch_size - 1;
                        const int dy = y - align_halfpatch_size - 1;
                        const int iix = ix + dx;
                        const int iiy = iy + dy;
                        if (iix < 0 || iiy < 0 || iix >= img_ref.cols - 1 || iiy >= img_ref.rows - 1) {
                            *patch_ptr = 0;
                        } else {
                            uchar *data = img_ref.data + iiy * img_ref.step + iix;
                            *patch_ptr =
                                    (1 - xx) * (1 - yy) * data[0] +
                                    xx * (1 - yy) * data[1] +
                                    (1 - xx) * yy * data[img_ref.step] +
                                    xx * yy * data[img_ref.step + 1];
                        }
                    }
                }

                // remove the boarder
                uint8_t *ref_patch_ptr = patch;
                for (int y = 1; y < align_patch_size + 1; ++y, ref_patch_ptr += align_patch_size) {
                    uint8_t *ref_patch_border_ptr = patch_with_border + y * (align_patch_size + 2) + 1;
                    for (int x = 0; x < align_patch_size; ++x)
                        ref_patch_ptr[x] = ref_patch_border_ptr[x];
                }

                bool ret = Align2D(
                        current->mPyramidLeft[lvl],
                        patch_with_border,
                        patch,
                        30,
                        posLvl
                );

                if (!keepNotConverged)
                    if (lvl == 2)
                        success = ret;

                // set the tracked pos
                trackedPos = posLvl * scale;

                if (trackedPos[0] < setting::boarder || trackedPos[0] >= setting::imageWidth - setting::boarder ||
                    trackedPos[1] < setting::boarder || trackedPos[1] >= setting::imageHeight - setting::boarder) {
                    success = false;
                    break;
                }
            }

            if (success) {
                // copy the results
                trackPts[i] = trackedPos;
                successPts++;
            } else {
                trackPts[i] = Vector2f(-1, -1);
            }
        }

        //std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        //timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
        //LOG(INFO)<<"Alignment cost time: "<<timeCost<<endl;
        return successPts;
    }

    bool LKFlowSinglePoint(
            const vector<Mat> &pyramid1,
            const vector<Mat> &pyramid2,
            const Vector2f &pixel1,
            Vector2f &pixel2
    ) {

        // 匹配局部地图用的 patch, 默认8x8
        uchar patch[align_patch_area] = {0};
        // 带边界的，左右各1个像素
        uchar patch_with_border[(align_patch_size + 2) * (align_patch_size + 2)] = {0};

        // from coarse to fine
        Vector2f refPixel = pixel1;
        Vector2f trackedPos = pixel2;
        bool success = true;

        for (int lvl = setting::numPyramid - 1; lvl >= 2; lvl--) {

            float scale = setting::scaleFactors[lvl];
            float invScale = setting::invScaleFactors[lvl];

            Vector2f posLvl = trackedPos * invScale;   // 第lvl层下的位置
            Vector2f refPixelLvl = refPixel * invScale;
            const cv::Mat &img_ref = pyramid1[lvl];

            // copy the patch with boarder
            uchar *patch_ptr = patch_with_border;
            const int ix = floor(refPixelLvl[0]);
            const int iy = floor(refPixelLvl[1]);
            const float xx = refPixelLvl[0] - ix;
            const float yy = refPixelLvl[1] - iy;

            for (int y = 0; y < align_patch_size + 2; y++) {
                for (int x = 0; x < align_patch_size + 2; x++, ++patch_ptr) {
                    const int dx = x - align_halfpatch_size - 1;
                    const int dy = y - align_halfpatch_size - 1;
                    const int iix = ix + dx;
                    const int iiy = iy + dy;
                    if (iix < 0 || iiy < 0 || iix >= img_ref.cols - 1 || iiy >= img_ref.rows - 1) {
                        *patch_ptr = 0;
                    } else {
                        uchar *data = img_ref.data + iiy * img_ref.step + iix;
                        *patch_ptr =
                                (1 - xx) * (1 - yy) * data[0] +
                                xx * (1 - yy) * data[1] +
                                (1 - xx) * yy * data[img_ref.step] +
                                xx * yy * data[img_ref.step + 1];
                    }
                }
            }

            // remove the boarder
            uint8_t *ref_patch_ptr = patch;
            for (int y = 1; y < align_patch_size + 1; ++y, ref_patch_ptr += align_patch_size) {
                uint8_t *ref_patch_border_ptr = patch_with_border + y * (align_patch_size + 2) + 1;
                for (int x = 0; x < align_patch_size; ++x)
                    ref_patch_ptr[x] = ref_patch_border_ptr[x];
            }

            bool ret = Align2D(
                    pyramid2[lvl],
                    patch_with_border,
                    patch,
                    30,
                    posLvl
            );

            if (lvl == 2)
                success = ret;

            // set the tracked pos
            trackedPos = posLvl * scale;

            if (trackedPos[0] < setting::boarder || trackedPos[0] >= setting::imageWidth - setting::boarder ||
                trackedPos[1] < setting::boarder || trackedPos[1] >= setting::imageHeight - setting::boarder) {
                success = false;
                break;
            }
        }

        if (success) {
            // copy the results
            pixel2 = trackedPos;
        }
        return success;
    }

    int LKFlow1D(const shared_ptr<Frame> frame) {

        // 匹配局部地图用的 patch, 默认8x8
        uchar patch[align_patch_area] = {0};
        // 带边界的，左右各1个像素
        uchar patch_with_border[(align_patch_size + 2) * (align_patch_size + 2)] = {0};

        int successPts = 0;
        for (shared_ptr<Feature> feat: frame->mFeaturesLeft) {
            // from coarse to fine
            Vector2f trackedPos = feat->mPixel;  // 第零层分辨率下的位置
            Vector2f refPixel = trackedPos;
            Vector2f direction(-1, 0);  // 右图中点应该出现在左侧
            bool success = true;

            for (int lvl = setting::numPyramid - 1; lvl >= 0; lvl--) {

                float scale = setting::scaleFactors[lvl];
                float invScale = setting::invScaleFactors[lvl];

                Vector2f posLvl = trackedPos * invScale;   // 第lvl层下的位置
                Vector2f refPixelLvl = refPixel * invScale;

                cv::Mat &img_ref = frame->mPyramidLeft[lvl];

                // copy the patch with boarder
                uchar *patch_ptr = patch_with_border;
                for (int y = 0; y < align_patch_size + 2; y++) {
                    for (int x = 0; x < align_patch_size + 2; x++, ++patch_ptr) {
                        Vector2f delta(x - align_halfpatch_size - 1, y - align_halfpatch_size - 1);
                        const Vector2f px(refPixelLvl + delta);
                        if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1) {
                            *patch_ptr = 0;
                        } else {
                            *patch_ptr = GetBilateralInterpUchar(px[0], px[1], img_ref);
                        }
                    }
                }

                // remove the boarder
                uint8_t *ref_patch_ptr = patch;
                for (int y = 1; y < align_patch_size + 1; ++y, ref_patch_ptr += align_patch_size) {
                    uint8_t *ref_patch_border_ptr = patch_with_border + y * (align_patch_size + 2) + 1;
                    for (int x = 0; x < align_patch_size; ++x)
                        ref_patch_ptr[x] = ref_patch_border_ptr[x];
                }

                /* 一维align的调用方法，但是对双目校正要求过高，不太现实
                double hinv = 0;
                success = Align1D(
                        frame->mPyramidRight[lvl],
                        direction,
                        patch_with_border,
                        patch,
                        10,
                        posLvl,
                        hinv
                );
                 */

                success = Align2D(
                        frame->mPyramidRight[lvl],
                        patch_with_border,
                        patch,
                        10,
                        posLvl
                );

                if (success == false)
                    break;
                // set the tracked pos
                trackedPos = posLvl * scale;
            }

            if (success) {
                // compute the disparity
                float disparity = refPixel[0] - trackedPos[0];
                feat->mfInvDepth = disparity / frame->mpCam->bf;

                successPts++;
            } else {
                feat->mfInvDepth = -1;
            }
        }

        return successPts;
    }

    int LKFlowCV(
            const shared_ptr<Frame> ref,
            const shared_ptr<Frame> current,
            VecVector2f &refPts,
            VecVector2f &trackedPts
    ) {
        if (refPts.size() == 0)
            return 0;

        vector<cv::Point2f> refPx, currPts;
        for (auto &px:refPts) {
            refPx.push_back(cv::Point2f(px[0], px[1]));
        }
        for (Vector2f &v: trackedPts) {
            currPts.push_back(cv::Point2f(v[0], v[1]));
        }

        vector<uchar> status;
        vector<float> err;

        cv::calcOpticalFlowPyrLK(ref->mImLeft, current->mImLeft, refPx, currPts, status, err,
                                 cv::Size(21, 21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        // reject with F matrix
        cv::findFundamentalMat(refPx, currPts, cv::FM_RANSAC, 3.0, 0.99, status);

        int successPts = 0;
        for (int i = 0; i < currPts.size(); i++) {
            if (status[i] && (currPts[i].x > setting::boarder && currPts[i].y > setting::boarder &&
                              currPts[i].x < setting::imageWidth - setting::boarder &&
                              currPts[i].y < setting::imageHeight - setting::boarder)) {
                // succeed
                // trackedPts.push_back(Vector2f(currPts[i].x, currPts[i].y));
                trackedPts[i] = Vector2f(currPts[i].x, currPts[i].y);
                successPts++;
            } else {
                // failed
                // trackedPts.push_back(Vector2f(-1, -1));
                trackedPts[i] = Vector2f(-1, -1);
            }
        }
        return successPts;
    }

}