#include "ygz/Feature.h"
#include "ygz/ORBMatcher.h"
#include "ygz/Frame.h"
#include "ygz/MapPoint.h"
#include "ygz/LKFlow.h"

#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace ygz {

    int ORBMatcher::DescriptorDistance(const uchar *desc1, const uchar *desc2) {
        int dist = 0;
        const int *pa = (int *) desc1;
        const int *pb = (int *) desc2;

        for (int i = 0; i < 8; i++, pa++, pb++) {
            unsigned int v = *pa ^*pb;
//#ifdef __SSE2__
//            dist += _mm_popcnt_u64(v);
//#else
            v = v - ( ( v >> 1 ) & 0x55555555 );
            v = ( v & 0x33333333 ) + ( ( v >> 2 ) & 0x33333333 );
            dist += ( ( ( v + ( v >> 4 ) ) & 0xF0F0F0F ) * 0x1010101 ) >> 24;
//#endif
        }
        return dist;
    }

    int ORBMatcher::SearchBruteForce(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, std::vector<Match> &matches) {
        assert (matches.empty());
        matches.reserve(frame1->mFeaturesLeft.size());

        for (size_t i = 0; i < frame1->mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> f1 = frame1->mFeaturesLeft[i];
            int min_dist = 9999;
            int min_dist_index = -1;

            for (size_t j = 0; j < frame2->mFeaturesLeft.size(); j++) {
                shared_ptr<Feature> f2 = frame2->mFeaturesLeft[j];
                int dist = ORBMatcher::DescriptorDistance(f1->mDesc, f2->mDesc);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_dist_index = j;
                }
            }

            if (min_dist < setting::TH_LOW) {
                matches.push_back(Match(i, min_dist_index, min_dist));
            }
        }

        return matches.size();
    }

    int ORBMatcher::SearchByBoW(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, std::vector<Match> &matches,
                                bool only3D) {

        const DBoW3::FeatureVector &vFeatVecF1 = frame1->mFeatVec;

        int nmatches = 0;

        vector<int> rotHist[setting::HISTO_LENGTH];
        for (int i = 0; i < setting::HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        DBoW3::FeatureVector::const_iterator F1it = vFeatVecF1.begin();
        DBoW3::FeatureVector::const_iterator F2it = frame2->mFeatVec.begin();
        DBoW3::FeatureVector::const_iterator F1end = vFeatVecF1.end();
        DBoW3::FeatureVector::const_iterator F2end = frame2->mFeatVec.end();

        while (F1it != F1end && F2it != F2end) {
            if (F1it->first == F2it->first) {
                const vector<unsigned int> vIndicesF1 = F1it->second;
                const vector<unsigned int> vIndicesF2 = F2it->second;

                for (size_t iKF = 0; iKF < vIndicesF1.size(); iKF++) {
                    const unsigned int realIdxF1 = vIndicesF1[iKF];

                    auto fea1 = frame1->mFeaturesLeft[realIdxF1];

                    if (only3D && fea1->mpPoint == nullptr)
                        continue;

                    const uchar *dF1 = fea1->mDesc;

                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    for (size_t iF = 0; iF < vIndicesF2.size(); iF++) {
                        const unsigned int realIdxF2 = vIndicesF2[iF];
                        auto fea2 = frame2->mFeaturesLeft[realIdxF2];

                        const uchar *dF2 = fea2->mDesc;

                        const int dist = DescriptorDistance(dF1, dF2);

                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF2;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= setting::TH_LOW) {
                        if (static_cast<float> ( bestDist1 ) < mfNNratio * static_cast<float> ( bestDist2 )) {
                            matches.push_back(Match(realIdxF1, bestIdxF, bestDist1));
                            nmatches++;
                        }
                    }

                }
                F1it++;
                F2it++;
            } else if (F1it->first < F2it->first) {
                F1it = vFeatVecF1.lower_bound(F2it->first);
            } else {
                F2it = frame2->mFeatVec.lower_bound(F1it->first);
            }
        }

        return nmatches;
    }

    int ORBMatcher::SearchByProjection(shared_ptr<Frame> CurrentFrame, shared_ptr<Frame> LastFrame, const float th) {

        int nmatches = 0;

        // Rotation Histogram (to check rotation consistency)
        vector<int> rotHist[setting::HISTO_LENGTH];
        for (int i = 0; i < setting::HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / setting::HISTO_LENGTH;

        const Matrix3d Rcw = CurrentFrame->mRcw;
        const Vector3d tcw = CurrentFrame->mtcw;
	
		
	// 上一帧的特征点遍历，对于每一个特征点，投影到成像平面上，
        for (size_t i = 0; i < LastFrame->mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> feature_last = LastFrame->mFeaturesLeft[i];
            shared_ptr<MapPoint> pMP = feature_last->mpPoint;

            if (pMP) {
                if (!feature_last->mbOutlier) {

                    // Project
                    Vector3d x3Dw = pMP->GetWorldPos();
                    Vector3d x3Dc = Rcw * x3Dw + tcw;

                    const float xc = x3Dc[0];
                    const float yc = x3Dc[1];
                    const float invzc = 1.0 / x3Dc[2];

                    if (invzc < 0)
                        continue;

                    float u = CurrentFrame->mpCam->fx * xc * invzc + CurrentFrame->mpCam->cx;
                    float v = CurrentFrame->mpCam->fy * yc * invzc + CurrentFrame->mpCam->cy;

                    // check 在图内
                    if (u < 0 || u >= setting::imageWidth)
                        continue;
                    if (v < 0 || v > setting::imageHeight)
                        continue;

                    int nLastOctave = feature_last->mLevel;

                    // Search in a window. Size depends on scale
                    float radius = th * setting::scaleFactors[nLastOctave];

                    vector<size_t> vIndices2;

                    // 获取候选点
                    vIndices2 = CurrentFrame->GetFeaturesInArea(u, v, radius, nLastOctave);

                    if (vIndices2.empty())
                        continue;

                    const uchar *dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end();
                         vit != vend; vit++) {

                        const size_t i2 = *vit;

                        if (CurrentFrame->mFeaturesLeft[i2]->mpPoint)
                            if (CurrentFrame->mFeaturesLeft[i2]->mpPoint->Observations() > 0)
                                continue;

                        // 如果已经有了右图的匹配，再检查右眼图像是否满足需要
                        if (CurrentFrame->mFeaturesLeft[i2]->mfInvDepth > 0) {
                            const float ur = u - CurrentFrame->mpCam->bf * invzc;
                            const float u2 = u - CurrentFrame->mpCam->bf * CurrentFrame->mFeaturesLeft[i2]->mfInvDepth;
                            const float er = fabs(ur - u2);
                            if (er > radius)
                                continue;
                        }

                        const uchar *d = CurrentFrame->mFeaturesLeft[i2]->mDesc;

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= setting::TH_HIGH) {
                        auto feature_curr = CurrentFrame->mFeaturesLeft[bestIdx2];
                        CurrentFrame->mFeaturesLeft[bestIdx2]->mpPoint = pMP;
                        nmatches++;

                        if (mbCheckOrientation) {
                            float rot = feature_last->mAngle - feature_curr->mAngle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == setting::HISTO_LENGTH)
                                bin = 0;
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
                }
            }
        }

        //Apply rotation consistency
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, setting::HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < setting::HISTO_LENGTH; i++) {
                if (i != ind1 && i != ind2 && i != ind3) {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                        CurrentFrame->mFeaturesLeft[rotHist[i][j]]->mpPoint = nullptr;
                        nmatches--;
                    }
                }
            }
        }
        return nmatches;
    }

    int ORBMatcher::SearchByProjection(shared_ptr<Frame> F, const std::set<shared_ptr<MapPoint>> &vpMapPoints,
                                       const float th) {

        assert (F != nullptr);
        int nmatches = 0;

        const bool bFactor = th != 1.0;

        int cntEmpty = 0;
        int cntInView = 0;
        for (auto pMP: vpMapPoints) {
            if (pMP->mbTrackInView == true) {
                cntInView++;
                continue;
            }

            if (pMP->isBad())
                continue;

            // const int &nPredictedLevel = pMP->mnTrackScaleLevel;

            // The size of the window will depend on the viewing direction
            float r = RadiusByViewingCos(pMP->mTrackViewCos);

            if (bFactor)
                r *= th;

            const vector<size_t> vIndices =
                    F->GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r,
                            // nPredictedLevel - 1, nPredictedLevel );
                                         0, setting::numPyramid);

            if (vIndices.empty()) {
                cntEmpty++;
                continue;
            }

            const uchar *MPdescriptor = pMP->GetDescriptor();

            int bestDist = 256;
            int bestLevel = -1;
            int bestDist2 = 256;
            int bestLevel2 = -1;
            int bestIdx = -1;

            // Get best and second matches with near keypoints
            for (auto vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;
                shared_ptr<MapPoint> mp = F->mFeaturesLeft[idx]->mpPoint;

                if (mp && mp->Observations() > 0) // 该点已经被匹配
                    continue;

                const uchar *d = F->mFeaturesLeft[idx]->mDesc;

                const int dist = DescriptorDistance(MPdescriptor, d);

                if (dist < bestDist) {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestLevel2 = bestLevel;
                    bestLevel = F->mFeaturesLeft[idx]->mLevel;
                    bestIdx = idx;
                } else if (dist < bestDist2) {
                    bestLevel2 = F->mFeaturesLeft[idx]->mLevel;
                    bestDist2 = dist;
                }
            }

            // Apply ratio to second match (only if best and second are in the same scale level)
            if (bestDist <= setting::TH_HIGH) {
                if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                    continue;
                F->mFeaturesLeft[bestIdx]->mpPoint = pMP;
                nmatches++;
            }
        }

        return nmatches;
    }

    void ORBMatcher::ComputeStereoMatches(shared_ptr<Frame> f, StereoMethod method) {
        if (method == ORB_BASED) {
            ComputeStereoMatchesORB(f);
        } else if (method == OPTIFLOW_BASED) {
            ComputeStereoMatchesOptiFlow(f);
        } else if (method == OPTIFLOW_CV) {
            ComputeStereoMatchesOptiFlowCV(f);
        }
    }

    void ORBMatcher::ComputeStereoMatchesORB(shared_ptr<Frame> f) {

        assert (f->mFeaturesLeft.size() != 0);
        assert (f->mFeaturesRight.size() != 0);

        const int thOrbDist = (setting::TH_HIGH + setting::TH_LOW) / 2;

        const int nRows = setting::imageWidth;

        // Assign keypoints to row table
        // 每行对应的右侧特征点
	
	// aaaaaaaaaaaaaaa!
        vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const size_t Nr = f->mFeaturesRight.size();

        for (size_t iR = 0; iR < Nr; iR++) {
            const shared_ptr<Feature> kp = f->mFeaturesRight[iR];
            const float &kpY = kp->mPixel[1];

            // 允许相差2个像素
            const float r = setting::stereoMatchingTolerance * setting::scaleFactors[kp->mLevel];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        const float minZ = f->mpCam->b;
        const float minD = 0;   //最小视差
        const float maxD = f->mpCam->bf / minZ; //最大视差

        // For each left keypoint search a match in the right image
        vector<pair<int, int>> vDistIdx;    // first = bestDist, second = iL
        size_t N = f->mFeaturesLeft.size();
        vDistIdx.reserve(N);

        for (size_t iL = 0; iL < N; iL++) {
            const shared_ptr<Feature> kpL = f->mFeaturesLeft[iL];
            const int levelL = kpL->mLevel;
            const float &vL = kpL->mPixel[1];
            const float &uL = kpL->mPixel[0];

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            int bestDist = setting::TH_HIGH;
            size_t bestIdxR = 0;

            const uchar *dL = f->mFeaturesLeft[iL]->mDesc;

            // Compare descriptor to right keypoints
            for (size_t iC = 0; iC < vCandidates.size(); iC++) {
                const size_t iR = vCandidates[iC];
                const shared_ptr<Feature> kpR = f->mFeaturesRight[iR];

                // 相差两层以上就不要了
                int rLevel = kpR->mLevel;
                if (rLevel < levelL - 1)
                    continue;
                if (rLevel > levelL + 1)
                    continue;

                const float &uR = kpR->mPixel[0];

                if (uR >= minU && uR <= maxU) {
                    // 落在合理的区间内，计算描述子距离
                    const uchar *dR = kpR->mDesc;
                    const int dist = ORBMatcher::DescriptorDistance(dL, dR);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist) {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = f->mFeaturesRight[bestIdxR]->mPixel[0];
                const float scaleFactor = setting::invScaleFactors[kpL->mLevel];
                const float scaleduL = round(kpL->mPixel[0] * scaleFactor);
                const float scaledvL = round(kpL->mPixel[1] * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                // 11x11 patch
                const int w = 5;
                cv::Mat IL = f->mPyramidLeft[kpL->mLevel].
                        rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
                IL.convertTo(IL, CV_32F);
                IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= f->mPyramidRight[kpL->mLevel].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++) {
                    cv::Mat IR = f->mPyramidRight[kpL->mLevel].rowRange(scaledvL - w, scaledvL + w + 1).colRange(
                            scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                    IR.convertTo(IR, CV_32F);
                    IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

                    // L1 距离
                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                float bestuR = setting::scaleFactors[kpL->mLevel] * ((float) scaleduR0 + (float) bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD) {
                    if (disparity <= 0) {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }

                    // disparity = f*b/z
                    f->mFeaturesLeft[iL]->mfInvDepth = disparity / f->mpCam->bf;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());

        // 谜之阈值，只取了dist较小的部分？
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        if (median == 0)
            return;

        for (int i = vDistIdx.size() - 1; i >= 0; i--) {
            if (vDistIdx[i].first < thDist) {
                break;
            } else {
                f->mFeaturesLeft[vDistIdx[i].second]->mfInvDepth = -1;
            }
        }
    }

    void ORBMatcher::ComputeStereoMatchesOptiFlow(shared_ptr<Frame> f, bool only2Dpoints) {
        //assert(!f->mFeaturesLeft.empty());
       	if(f->mFeaturesLeft.empty())
	    return;	

	if (f->mPyramidLeft.empty() || f->mPyramidRight.empty())
            f->ComputeImagePyramid();
        // 对于那些未关联地图点的特征，或关联了未成熟地图点的特征，尝试通过双目估计其深度
        for (int i = 0; i < f->mFeaturesLeft.size(); i++) {
            auto &feat = f->mFeaturesLeft[i];
            if (feat == nullptr)
                continue;
            if (only2Dpoints && feat->mpPoint &&
                feat->mpPoint->Status() == MapPoint::GOOD)    // already have a good point
                continue;
            Vector2f pl = feat->mPixel;
            Vector2f pr = feat->mPixel;
            bool ret = LKFlowSinglePoint(f->mPyramidLeft, f->mPyramidRight, feat->mPixel, pr);

            if (ret) {
                // check the right one
                if (pl[0] < pr[0] || (fabs(pl[1] - pr[1]) > setting::stereoMatchingTolerance)) {
                    continue;
                } else {
                    float disparity = pl[0] - pr[0];
                    if (disparity > 1)    // avoid zero disparity
                        feat->mfInvDepth = disparity / f->mpCam->bf;
                }
            }
        }
    }

    void ORBMatcher::ComputeStereoMatchesOptiFlowCV(shared_ptr<Frame> f) {

        vector<cv::Point2f> leftPts, rightPts;
        vector<shared_ptr<Feature>> validFeats;
        for (auto feat: f->mFeaturesLeft) {
            if (feat->mpPoint == nullptr) {
                leftPts.push_back(cv::Point2f(feat->mPixel[0], feat->mPixel[1]));
                validFeats.push_back(feat);
            }
        }

        if (leftPts.empty())
            return;

        vector<uchar> status;
        vector<float> error;
        cv::calcOpticalFlowPyrLK(f->mImLeft, f->mImRight, leftPts, rightPts, status, error,
                                 cv::Size(21, 21), 3);

        for (size_t i = 0; i < rightPts.size(); i++) {
            if (status[i]) {
                // lk succeed
                shared_ptr<Feature> &feat = validFeats[i];
                cv::Point2f &pl = leftPts[i];
                cv::Point2f &pr = rightPts[i];
                if (pl.x < pr.x || (fabs(pl.y - pr.y) > 5))   // x or y is not right
                    continue;
                float disparity = pl.x - pr.x;
                feat->mfInvDepth = disparity / f->mpCam->bf;
            }
        }
    }

    int ORBMatcher::SearchByDirectProjection(shared_ptr<Frame> F, const std::set<shared_ptr<MapPoint>> &vpMapPoints) {

        // unique_lock<mutex> lock(F->mMutexFeature);
        F->AssignFeaturesToGrid();

        // 匹配局部地图用的 patch, 默认8x8
        uchar patch[align_patch_area] = {0};
        // 带边界的，左右各1个像素
        uchar patch_with_border[(align_patch_size + 2) * (align_patch_size + 2)] = {0};

        int cntSucceed = 0;
        for (const shared_ptr<MapPoint> &mp: vpMapPoints) {
            if (mp == nullptr)
                continue;

            if (mp->mpRefKF.expired()) continue;
            shared_ptr<Frame> kf = mp->mpRefKF.lock();

            if (mp->mbTrackInView) continue;

            if (!F->GetFeaturesInArea(mp->mTrackProjX, mp->mTrackProjY, 20).empty()) {
                // there is already a matched point here
                continue;
            }

            // Try align this map point
            unique_lock<mutex> lock(kf->mMutexFeature);
            size_t idxFeatKF = mp->GetObsFromKF(kf);
            if (idxFeatKF < 0 || idxFeatKF >= kf->mFeaturesLeft.size())
                continue;

            shared_ptr<Feature> refFeat = kf->mFeaturesLeft[idxFeatKF];
            if (refFeat == nullptr)
                continue;

            Eigen::Matrix2d ACR;
            Vector2f px_ref = refFeat->mPixel;
            SE3d pose_ref = SE3d(kf->Rcw(), kf->Tcw());
            SE3d TCR = SE3d(F->Rcw(), F->Tcw()) * pose_ref.inverse();

            // 计算带边界的affine wrap，边界是为了便于计算梯度
            this->GetWarpAffineMatrix(kf, F, refFeat, refFeat->mLevel, TCR, ACR);

            int search_level = 0;

            WarpAffine(ACR, kf->mImLeft, px_ref.cast<double>(), 0, kf, 0, align_halfpatch_size + 1,
                       patch_with_border);

            // remove the boarder
            uint8_t *ref_patch_ptr = patch;
            for (int y = 1; y < align_patch_size + 1; ++y, ref_patch_ptr += align_patch_size) {
                uint8_t *ref_patch_border_ptr = patch_with_border + y * (align_patch_size + 2) + 1;
                for (int x = 0; x < align_patch_size; ++x)
                    ref_patch_ptr[x] = ref_patch_border_ptr[x];
            }

            Vector2f px_curr(mp->mTrackProjX, mp->mTrackProjY);
            Vector2f px_scaled = px_curr * setting::invScaleFactors[refFeat->mLevel];

            bool success = Align2D(F->mImLeft, patch_with_border, patch, 10, px_scaled);
            px_curr = px_scaled * setting::scaleFactors[search_level];

            if (success) {
                // Create a feature in current
                shared_ptr<Feature> feat(new Feature);
                feat->mPixel = px_curr;
                feat->mpPoint = mp;
                feat->mLevel = search_level;
                F->mFeaturesLeft.push_back(feat);
                cntSucceed++;
            }
        }

        return cntSucceed;

    }

    void ORBMatcher::GetWarpAffineMatrix(shared_ptr<Frame> ref, shared_ptr<Frame> curr, const shared_ptr<Feature> feat,
                                         int level, const SE3d &TCR, Eigen::Matrix2d &ACR) {

        float depth = 1.0 / feat->mfInvDepth;
        Vector3d pt_ref = ref->Pixel2Camera(feat->mPixel.cast<double>(), depth);

        // 偏移之后的3d点，深度取成和pt_ref一致
        const Vector3d pt_du_ref = ref->Pixel2Camera(
                feat->mPixel.cast<double>() + Vector2d(align_halfpatch_size, 0) * (double) setting::scaleFactors[level],
                depth);
        const Vector3d pt_dv_ref = ref->Pixel2Camera(
                feat->mPixel.cast<double>() + Vector2d(0, align_halfpatch_size) * (double) setting::scaleFactors[level],
                depth);

        const Vector2d px_cur = curr->World2Pixel(pt_ref, TCR);
        const Vector2d px_du = curr->World2Pixel(pt_du_ref, TCR);
        const Vector2d px_dv = curr->World2Pixel(pt_dv_ref, TCR);

        ACR.col(0) = (px_du - px_cur) /
                     align_halfpatch_size;
        ACR.col(1) = (px_dv - px_cur) /
                     align_halfpatch_size;
    }

    void ORBMatcher::WarpAffine(
            const Matrix2d &ACR, const Mat &img_ref,
            const Vector2d &px_ref, const int &level_ref, const shared_ptr<Frame> ref,
            const int &search_level, const int &half_patch_size, uint8_t *patch) {

        const int patch_size = half_patch_size * 2;
        const Eigen::Matrix2d ARC = ACR.inverse();

        // Affine warp
        uint8_t *patch_ptr = patch;
        const Vector2d px_ref_pyr = px_ref / setting::scaleFactors[level_ref];
        for (int y = 0; y < patch_size; y++) {
            for (int x = 0; x < patch_size; x++, ++patch_ptr) {
                Vector2d px_patch(x - half_patch_size, y - half_patch_size);
                px_patch *= setting::scaleFactors[search_level];
                const Vector2d px(ARC * px_patch + px_ref_pyr);
                if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1) {
                    *patch_ptr = 0;
                } else {
                    *patch_ptr = GetBilateralInterpUchar(px[0], px[1], img_ref);
                }
            }
        }
    }

    void ORBMatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3) {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; i++) {
            const int s = histo[i].size();
            if (s > max1) {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            } else if (s > max2) {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            } else if (s > max3) {
                max3 = s;
                ind3 = i;
            }
        }

        if (max2 < 0.1f * (float) max1) {
            ind2 = -1;
            ind3 = -1;
        } else if (max3 < 0.1f * (float) max1) {
            ind3 = -1;
        }
    }

    void ORBMatcher::ComputeDistinctiveDescriptors(shared_ptr<MapPoint> mp) {
        // Retrieve all observed descriptors
        // 获取所有描述
        auto observations = mp->GetObservations();

        if (observations.empty())
            return;

        vector<uchar *> vDescriptors;
        vDescriptors.reserve(observations.size());

        for (auto &obs: observations) {
            weak_ptr<Frame> pKF = obs.first;
            if (pKF.expired())
                continue;

            uchar *desc = pKF.lock()->mFeaturesLeft[obs.second]->mDesc;
            vDescriptors.push_back(desc);
        }

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++) {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBMatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = 9999;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            memcpy(mp->mDescriptor, vDescriptors[BestIdx], 32);
        }
    }

    int ORBMatcher::SearchForTriangulation(shared_ptr<Frame> pKF1, shared_ptr<Frame> pKF2, Matrix3d &F12,
                                           std::vector<Match> &vMatchedPairs) {

        const DBoW3::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW3::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches = 0;
        vector<bool> vbMatched2(pKF2->mFeaturesLeft.size(), false);
        vector<int> vMatches12(pKF1->mFeaturesLeft.size(), -1);

        vector<int> rotHist[setting::HISTO_LENGTH];
        for (int i = 0; i < setting::HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f / setting::HISTO_LENGTH;

        DBoW3::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW3::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW3::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW3::FeatureVector::const_iterator f2end = vFeatVec2.end();

        while (f1it != f1end && f2it != f2end) {
            if (f1it->first == f2it->first) {
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                    const size_t idx1 = f1it->second[i1];

                    shared_ptr<MapPoint> pMP1 = pKF1->mFeaturesLeft[idx1]->mpPoint;

                    // If there is already a MapPoint skip
                    if (pMP1)
                        continue;

                    const shared_ptr<Feature> feat1 = pKF1->mFeaturesLeft[idx1];
                    const uchar *d1 = feat1->mDesc;

                    int bestDist = setting::TH_LOW;
                    int bestIdx2 = -1;

                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                        size_t idx2 = f2it->second[i2];

                        shared_ptr<Feature> feat2 = pKF2->mFeaturesLeft[idx2];
                        shared_ptr<MapPoint> pMP2 = feat2->mpPoint;

                        // If we have already matched or there is a MapPoint skip
                        if (vbMatched2[idx2] || pMP2)
                            continue;

                        const uchar *d2 = feat2->mDesc;

                        const int dist = DescriptorDistance(d1, d2);

                        if (dist > setting::TH_LOW || dist > bestDist)
                            continue;

                        if (CheckDistEpipolarLine(feat1, feat2, F12)) {
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }

                    if (bestIdx2 >= 0) {
                        const shared_ptr<Feature> feat2 = pKF2->mFeaturesLeft[bestIdx2];
                        vMatches12[idx1] = bestIdx2;
                        nmatches++;

                        if (mbCheckOrientation) {
                            float rot = feat1->mAngle - feat2->mAngle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == setting::HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < setting::HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            } else if (f1it->first < f2it->first) {
                f1it = vFeatVec1.lower_bound(f2it->first);
            } else {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, setting::HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < setting::HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vMatches12[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }

        }

        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
            if (vMatches12[i] < 0)
                continue;
            vMatchedPairs.push_back(Match(i, vMatches12[i], -1));
        }

        return nmatches;
    }

    bool ORBMatcher::CheckDistEpipolarLine(const shared_ptr<Feature> feat1, const shared_ptr<Feature> feat2,
                                           const Matrix3d &F12) {

        // Epipolar line in second image l = x1'F12 = [a b c]
        const float a = feat1->mPixel[0] * F12(0, 0) + feat1->mPixel[1] * F12(1, 0) + F12(2, 0);
        const float b = feat1->mPixel[0] * F12(0, 1) + feat1->mPixel[1] * F12(1, 1) + F12(2, 1);
        const float c = feat1->mPixel[0] * F12(0, 2) + feat1->mPixel[1] * F12(1, 2) + F12(2, 2);
        const float num = a * feat2->mPixel[0] + b * feat2->mPixel[1] + c;
        const float den = a * a + b * b;
        if (den == 0)
            return false;
        const float dsqr = num * num / den;
        return dsqr < 3.84 * setting::levelSigma2[feat2->mLevel];
    }


}
