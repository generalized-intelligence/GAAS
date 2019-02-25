#include "ygz/MapPoint.h"
#include "ygz/Frame.h"
#include "ygz/Feature.h"
#include "ygz/ORBMatcher.h"

#include <thread>
#include <mutex>

using namespace std;

namespace ygz {

    long unsigned int MapPoint::nNextId = 0;

    // 从关键帧新建地图点
    MapPoint::MapPoint(shared_ptr<Frame> pFrame, const size_t &idxF) : mpRefKF(pFrame) {
        assert(pFrame != nullptr);
        assert(idxF < pFrame->mFeaturesLeft.size() && idxF >= 0);
        shared_ptr<Feature> feat = pFrame->mFeaturesLeft[idxF];

        // 计算该特征点的投影
        Vector3d ptFrame = pFrame->mpCam->Img2Cam(feat->mPixel) * (1.0 / double(feat->mfInvDepth));
        Vector3d Ow = pFrame->Ow();
        mWorldPos = pFrame->Rwc() * ptFrame + Ow;

        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / mNormalVector.norm();

        for (size_t i = 0; i < 32; i++)
            mDescriptor[i] = pFrame->mFeaturesLeft[idxF]->mDesc[i];

        mnId = nNextId++;
        mGray = pFrame->mImLeft.data[int(feat->mPixel[1]) * pFrame->mImLeft.cols + int(feat->mPixel[0])];
    }

    MapPoint::~MapPoint() {

    }

    void MapPoint::ComputeDistinctiveDescriptor() {
        // Retrieve all observed descriptors
        vector<uchar *> vDescriptors;
        if (mObservations.empty())
            return;

        vDescriptors.reserve(mObservations.size());

        for (auto mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++) {
            weak_ptr<Frame> pKF = mit->first;
            if (pKF.expired() == false)
                vDescriptors.push_back(pKF.lock()->mFeaturesLeft[mit->second]->mDesc);
        }

        if (vDescriptors.empty())
            return;

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
        int BestMedian = INT_MAX;
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
            unique_lock<mutex> lock(mMutexFeatures);
            memcpy(mDescriptor, vDescriptors[BestIdx], 32);
        }
    }

    bool MapPoint::SetAnotherRef() {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        for (auto &obs: mObservations) {
            if (obs.first.expired() == false && obs.first.lock() != mpRefKF.lock()) {
                mpRefKF = obs.first;
                return true;
            }
        }
        return false;
    }

    int MapPoint::GetObsFromKF(shared_ptr<Frame> pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        weak_ptr<Frame> idx(pKF);
        auto iter = mObservations.find(idx);
        if (iter == mObservations.end())
            return -1;
        return iter->second;
    }

    bool MapPoint::RemoveObservation(shared_ptr<Frame> &pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        mObservations.erase(pKF);
        return true;
    }

    bool MapPoint::RemoveObservation(weak_ptr<Frame> &pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        mObservations.erase(pKF);
        return true;
    }

    int MapPoint::Observations() {
        unique_lock<mutex> lock(mMutexFeatures);
        int cnt = 0;
        for (ObsMap::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++) {
            if (!mit->first.expired()) {
                cnt++;
            }
        }
        return cnt;
    }

    void MapPoint::SetWorldPos(const Vector3d &Pos) {
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    void MapPoint::UpdateWorldPos() {
        if (mpRefKF.expired() == false) {
            unique_lock<mutex> lock(mMutexPos);
            mWorldPos = mpRefKF.lock()->UnprojectStereo(mObservations[mpRefKF]);
        }
    }

    Vector3d MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    Vector3d MapPoint::GetNormal() {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }

    void MapPoint::AddObservation(shared_ptr<Frame> pKF, size_t idx) {
        assert(pKF);
        assert(pKF->mbIsKeyFrame);
        assert(idx < pKF->mFeaturesLeft.size());
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
    }

    MapPoint::ObsMap MapPoint::GetObservations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    void MapPoint::SetBadFlag() {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mState = BAD;
    }

    void MapPoint::IncreaseVisible(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::SetStatus(eMapPointState state) {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mState = state;
    }

    void MapPoint::IncreaseFound(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }


    void MapPoint::UpdateNormalAndDepth() {
        // 更新地图点的法线和深度
        if (mpRefKF.expired())
            return;

        unique_lock<mutex> lock(mMutexFeatures);
        Vector3d normal(0, 0, 0);
        shared_ptr<Frame> kf = mpRefKF.lock();
        mNormalVector = mWorldPos - kf->Ow();
        mNormalVector.normalize();
    }

    void MapPoint::CheckReprojection() {

        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        UpdateWorldPos();
        for (auto &obs: mObservations) {
            if (obs.first.expired() == false) {
                auto frame = obs.first.lock();
                auto feat = frame->mFeaturesLeft[obs.second];
                LOG(INFO) << "observed pixel: " << feat->mPixel.transpose() << endl;
                Vector2d px = frame->World2Pixel(mWorldPos, SE3d(frame->mRcw, frame->mtcw));
                LOG(INFO) << "projected pixel: " << px.transpose() << endl;
            }
        }
    }

    bool MapPoint::TestGoodFromImmature() {
        if (mObservations.size() == 0)
            return false;

        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);

        // 成为一个好点，必须有足够的视线角之差
        if (mpRefKF.expired())
            return false;
        shared_ptr<Frame> refKF = mpRefKF.lock();
        shared_ptr<Feature> feat1 = refKF->mFeaturesLeft[mObservations[mpRefKF]];

        shared_ptr<CameraParam> cam = refKF->mpCam;
        const float &fx1 = cam->fx;
        const float &fy1 = cam->fy;
        const float &cx1 = cam->cx;
        const float &cy1 = cam->cy;
        const float &invfx1 = cam->fxinv;
        const float &invfy1 = cam->fyinv;

        Matrix3d Rcw1 = refKF->Rcw();
        Matrix3d Rwc1 = refKF->Rwc();
        Vector3d tcw1 = refKF->Tcw();
        Eigen::Matrix<double, 3, 4> Tcw1;
        Tcw1.block<3, 3>(0, 0) = Rcw1;
        Tcw1.block<3, 1>(0, 3) = tcw1;

        for (auto &obs: mObservations) {
            if (obs.first.expired())
                continue;
            shared_ptr<Frame> kf = obs.first.lock();
            if (kf == refKF)
                continue;
            shared_ptr<Feature> feat2 = kf->mFeaturesLeft[obs.second];

            Matrix3d Rcw2 = kf->Rcw();
            Matrix3d Rwc2 = kf->Rwc();
            Vector3d tcw2 = kf->Tcw();

            Matrix<double, 3, 4> Tcw2;
            Tcw2.block<3, 3>(0, 0) = Rcw2;
            Tcw2.block<3, 1>(0, 3) = tcw2;

            Vector3d xn1((feat1->mPixel[0] - cx1) * invfx1, (feat1->mPixel[1] - cy1) * invfy1, 1.0);
            Vector3d xn2((feat2->mPixel[0] - cx1) * invfx1, (feat2->mPixel[1] - cy1) * invfy1, 1.0);

            Vector3d ray1 = Rwc1 * xn1;
            Vector3d ray2 = Rwc2 * xn2;

            const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

            Vector3d x3D;
            if (cosParallaxRays > 0 && cosParallaxRays < 0.9998) {
                // 视角足够
                Matrix4d A;
                A.row(0) = xn1[0] * Tcw1.row(2) - Tcw1.row(0);
                A.row(1) = xn1[1] * Tcw1.row(2) - Tcw1.row(1);
                A.row(2) = xn2[0] * Tcw2.row(2) - Tcw2.row(0);
                A.row(3) = xn2[1] * Tcw2.row(2) - Tcw2.row(1);

                if (std::abs(A.determinant()) < 1e-4)
                    continue;

                Eigen::JacobiSVD<Matrix4d> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Matrix4d V = SVD.matrixV();
                if (V(3, 3) == 0)
                    continue;
                x3D = V.block<3, 1>(0, 3) / V(3, 3);
            } else
                continue;

            float z1 = Rcw1.row(2).dot(x3D) + tcw1[2];
            if (z1 <= 0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2[2];
            if (z2 <= 0)
                continue;

            const float &sigmaSquare1 = setting::levelSigma2[feat1->mLevel];
            const float x1 = Rcw1.row(0).dot(x3D) + tcw1[0];
            const float y1 = Rcw1.row(1).dot(x3D) + tcw1[1];
            const float invz1 = 1.0 / z1;

            float u1 = fx1 * x1 * invz1 + cx1;
            float v1 = fy1 * y1 * invz1 + cy1;
            float errX1 = u1 - feat1->mPixel[0];
            float errY1 = v1 - feat1->mPixel[1];
            if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                continue;

            // Check reprojection error in second keyframe
            const float sigmaSquare2 = setting::levelSigma2[feat2->mLevel];
            const float x2 = Rcw2.row(0).dot(x3D) + tcw2[0];
            const float y2 = Rcw2.row(1).dot(x3D) + tcw2[1];
            const float invz2 = 1.0 / z2;
            float u2 = fx1 * x2 * invz2 + cx1;
            float v2 = fy1 * y2 * invz2 + cy1;
            float errX2 = u2 - feat2->mPixel[0];
            float errY2 = v2 - feat2->mPixel[1];
            if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                continue;

            feat1->mfInvDepth = invz1;
            feat2->mfInvDepth = invz2;

            mWorldPos = x3D;
            return true;
        }
        return false;
    }

}
