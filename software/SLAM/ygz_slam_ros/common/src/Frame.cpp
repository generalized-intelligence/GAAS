#include "ygz/Frame.h"
// #include <experimental/filesystem>

using namespace cv;


namespace ygz {

    // static variables
    long unsigned int Frame::nNextId = 0;
    long unsigned int Frame::nNextKFId = 0;
    shared_ptr<ORBVocabulary> Frame::pORBvocabulary = nullptr;
    
    
    void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
    {
        m_brief.compute(im, keys, descriptors);
    }
    
    BriefExtractor::BriefExtractor(const std::string &pattern_file)
    {
        // The DVision::BRIEF extractor computes a random pattern by default when
        // the object is created.
        // We load the pattern that we used to build the vocabulary, to make
        // the descriptors compatible with the predefined vocabulary

        // loads the pattern
        cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
        if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

        vector<int> x1, y1, x2, y2;
        fs["x1"] >> x1;
        fs["x2"] >> x2;
        fs["y1"] >> y1;
        fs["y2"] >> y2;

        m_brief.importPairs(x1, y1, x2, y2);
    }

    // copy constructor
    Frame::Frame(const Frame &frame)
            :
            mTimeStamp(frame.mTimeStamp), mpCam(frame.mpCam),
            mFeaturesLeft(frame.mFeaturesLeft), mFeaturesRight(frame.mFeaturesRight), mnId(frame.mnId),
            mpReferenceKF(frame.mpReferenceKF), mImLeft(frame.mImLeft), mImRight(frame.mImRight) {
        SetPose(SE3d(frame.mRwb, frame.mTwb));
        mGrid.resize(setting::FRAME_GRID_ROWS * setting::FRAME_GRID_COLS);
    }

    // normal constructor
    Frame::Frame(const cv::Mat &left, const cv::Mat &right, const double &timestamp, shared_ptr<CameraParam> cam,
                 const VecIMU &IMUSinceLastFrame,const Vector3d gps_xyz,const Matrix3d compass_attitude,bool use_gps_and_compass)
            : mTimeStamp(timestamp), mImLeft(left.clone()), mImRight(right.clone()), mpCam(cam),
              mvIMUDataSinceLastFrame(IMUSinceLastFrame) {
        this->muse_compass_and_gps=use_gps_and_compass;
        if (use_gps_and_compass)
        {
            this->mgps_xyz = gps_xyz;
            this->mcompass_attitude=compass_attitude;
            SetPose(SE3d(this->mcompass_attitude,this->mgps_xyz));
        }
        else
        {
            SetPose(SE3d());
        }
        mnId = nNextId++;
        mGrid.resize(setting::FRAME_GRID_ROWS * setting::FRAME_GRID_COLS);
    }

    Frame::~Frame() {
        mFeaturesLeft.clear();
        mFeaturesRight.clear();
    }

    void Frame::SetPose(const SE3d &Twb) {
        unique_lock<mutex> lock(mMutexPose);
        mRwb = Twb.rotationMatrix();
        mTwb = Twb.translation();
        SE3d TWC = Twb * setting::TBC;
        SE3d TCW = TWC.inverse();
        mRcw = TCW.rotationMatrix();
        mtcw = TCW.translation();
        mOw = TWC.translation();
        mRwc = TWC.rotationMatrix();
    }
    
    void Frame::SetPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &T)
    {
        unique_lock<mutex> lock(mMutexPose);
        mRwb = SO3d(R);
        mTwb = T;
        SE3d Twb = SE3d(mRwb, mTwb);
        SE3d TWC = Twb * setting::TBC;
        SE3d TCW = TWC.inverse();
        mRcw = TCW.rotationMatrix();
        mtcw = TCW.translation();
        mOw = TWC.translation();
        mRwc = TWC.rotationMatrix();
    }

    
    void Frame::SetPoseTCW(const SE3d &Tcw) {
        unique_lock<mutex> lock(mMutexPose);
        mRcw = Tcw.rotationMatrix();
        mtcw = Tcw.translation();
        SE3d Twc = Tcw.inverse();
        mRwc = Twc.rotationMatrix();
        mOw = Twc.translation();
        SE3d Twb = Twc * setting::TBC.inverse();
        mRwb = Twb.rotationMatrix();
        mTwb = Twb.translation();
    }

    bool Frame::SetThisAsKeyFrame() {
        if (mbIsKeyFrame == true)
            return true;

        // 置关键帧
        mbIsKeyFrame = true;
        mnKFId = nNextKFId++;
        return true;
    }

    void Frame::ComputeIMUPreIntSinceLastFrame(const shared_ptr<Frame> pLastF, IMUPreIntegration &IMUPreInt) {
        // Reset pre-integrator first
        unique_lock<mutex> lock(mMutexPose);
        IMUPreInt.reset();
        //cout <<"Debug 3-1"<<endl;
        const VecIMU &vIMUSInceLastFrame = mvIMUDataSinceLastFrame;

        Vector3d bg = pLastF->BiasG();
        Vector3d ba = pLastF->BiasA();
        //cout <<"Debug 3-2"<<endl;
        // remember to consider the gap between the last KF and the first IMU
	if (vIMUSInceLastFrame.size()>0)
        {
            const IMUData &imu = vIMUSInceLastFrame.front();
            double dt = std::max(0., imu.mfTimeStamp - pLastF->mTimeStamp);
            IMUPreInt.update(imu.mfGyro - bg, imu.mfAcce - ba, dt);
        }
        // integrate each imu
        //cout <<"Debug 3-3"<<endl;
        for (size_t i = 0; i < vIMUSInceLastFrame.size(); i++) {
            const IMUData &imu = vIMUSInceLastFrame[i];
	    //cout <<"Debug 3-4"<<endl;
            double nextt;
            if (i == vIMUSInceLastFrame.size() - 1)
	    {
	        //cout <<"Debug 3-5"<<endl;
                nextt = mTimeStamp;         // last IMU, next is this KeyFrame
	    }
            else
	    {
	        //cout <<"Debug 3-6"<<endl;
                nextt = vIMUSInceLastFrame[i + 1].mfTimeStamp;  // regular condition, next is imu data
	    }
            //cout <<"Debug 3-7"<<endl;
            // delta time
            double dt = std::max(0., nextt - imu.mfTimeStamp);
	    //cout <<"Debug 3-8"<<endl;
            // update pre-integrator
            IMUPreInt.update(imu.mfGyro - bg, imu.mfAcce - ba, dt);
	    //cout <<"Debug 3-9"<<endl;
        }
    }

    void Frame::UpdatePoseFromPreintegration(const IMUPreIntegration &imupreint, const Vector3d &gw) {
        unique_lock<mutex> lock(mMutexPose);

        Matrix3d dR = imupreint.getDeltaR();
        Vector3d dP = imupreint.getDeltaP();
        Vector3d dV = imupreint.getDeltaV();
        double dt = imupreint.getDeltaTime();

        Vector3d Pwbpre = mTwb;     // 平移
        Matrix3d Rwbpre = mRwb.matrix();
        Vector3d Vwbpre = mSpeedAndBias.segment<3>(0);

        Matrix3d Rwb = Rwbpre * dR;
        Vector3d Pwb = Pwbpre + Vwbpre * dt + 0.5 * gw * dt * dt + Rwbpre * dP;
        Vector3d Vwb = Vwbpre + gw * dt + Rwbpre * dV;

        // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
        mRwb = Rwb;
        mTwb = Pwb;
        SE3d TWC = SE3d(Rwb, Pwb) * setting::TBC;
        SE3d TCW = TWC.inverse();
        mRcw = TCW.rotationMatrix();
        mtcw = TCW.translation();
        mOw = TWC.translation();
        mRwc = TWC.rotationMatrix();
        mSpeedAndBias.segment<3>(0) = Vwb;
    }

    vector<size_t> Frame::GetFeaturesInArea(
            const float &x, const float &y, const float &r, const int minLevel,
            const int maxLevel) {
        unique_lock<mutex> lock(mMutexFeature);
        vector<size_t> vIndices;
        vIndices.reserve(mFeaturesLeft.size());

        const int nMinCellX = max(0, (int) floor((x - r) * setting::GridElementWidthInv));
        if (nMinCellX >= setting::FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int) setting::FRAME_GRID_COLS - 1,
                                  (int) ceil((x + r) * setting::GridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - r) * setting::GridElementHeightInv));
        if (nMinCellY >= setting::FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int) setting::FRAME_GRID_ROWS - 1,
                                  (int) ceil((y + r) * setting::GridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[iy * setting::FRAME_GRID_COLS + ix];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {

                    const shared_ptr<Feature> feature = mFeaturesLeft[vCell[j]];
                    if (bCheckLevels) {
                        if (int(feature->mLevel) < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (int(feature->mLevel) > maxLevel)
                                continue;
                    }

                    const float distx = feature->mPixel[0] - x;
                    const float disty = feature->mPixel[1] - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }
        return vIndices;
    }

    void Frame::AssignFeaturesToGrid() {
        if (mFeaturesLeft.empty())
            return;

        for (auto g: mGrid)
            g.clear();

        unique_lock<mutex> lock(mMutexFeature);
        for (size_t i = 0; i < mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> f = mFeaturesLeft[i];
            if (f == nullptr)
                continue;
            int nGridPosX, nGridPosY;
            if (PosInGrid(f, nGridPosX, nGridPosY)) {
                mGrid[nGridPosX + nGridPosY * setting::FRAME_GRID_COLS].push_back(i);
            }
        }
    }

    bool Frame::isInFrustum(shared_ptr<MapPoint> pMP, float viewingCosLimit, int boarder) {//is inside of valid range(angle and distance)

        // 3D in absolute coordinates
        Vector3d P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Vector3d Pc = mRcw * P + mtcw;
        const float &PcX = Pc[0];
        const float &PcY = Pc[1];
        const float &PcZ = Pc[2];

        // Check valid depth
        if (PcZ < setting::minPointDis || PcZ > setting::maxPointDis) {
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        const float u = mpCam->fx * PcX * invz + mpCam->cx;
        const float v = mpCam->fy * PcY * invz + mpCam->cy;

        if (u < boarder || u > (setting::imageWidth - boarder))
            return false;
        if (v < boarder || v > (setting::imageHeight - boarder))
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const Vector3d PO = P - mOw;
        const float dist = PO.norm();

        // Check viewing angle
        Vector3d Pn = pMP->GetNormal();
        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit) {
            return false;
        }

        // Data used by the tracking
        pMP->mTrackProjX = u;
        pMP->mTrackProjY = v;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    void Frame::ComputeBoW() {
        if (pORBvocabulary && mBowVec.empty() && !mFeaturesLeft.empty()) {
            vector<cv::Mat> vCurrentDesc = this->GetAllDescriptor();
            pORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    vector<Mat> Frame::GetAllDescriptor() {
        unique_lock<mutex> lock(mMutexFeature);
        vector<Mat> ret;
        ret.reserve(mFeaturesLeft.size());
        for (size_t i = 0; i < mFeaturesLeft.size(); i++) {
            ret.push_back(cv::Mat(1, 32, CV_8UC1, mFeaturesLeft[i]->mDesc));
        }
        return ret;
    }

    Vector3d Frame::UnprojectStereo(const int &i) {
        unique_lock<mutex> lock(mMutexFeature);

        const float z = 1.0 / mFeaturesLeft[i]->mfInvDepth;
        if (z > 0) {
            const float u = mFeaturesLeft[i]->mPixel[0];
            const float v = mFeaturesLeft[i]->mPixel[1];
            const float x = (u - mpCam->cx) * z * mpCam->fxinv;
            const float y = (v - mpCam->cy) * z * mpCam->fyinv;
            Vector3d x3Dc(x, y, z);
            return mRwc * x3Dc + mOw;
        } else
            return Vector3d(0, 0, 0);
    }

    void Frame::ComputeImagePyramid() {

        // vector<cv::Mat> mPyramidLeft
        mPyramidLeft.resize(setting::numPyramid);
        mPyramidRight.resize(setting::numPyramid);

        for (size_t level = 0; level < setting::numPyramid; ++level)
        {
            float scale = setting::invScaleFactors[level];
            Size sz(cvRound((float) mImLeft.cols * scale), cvRound((float) mImLeft.rows * scale));
            Size wholeSize(sz.width + setting::EDGE_THRESHOLD * 2, sz.height + setting::EDGE_THRESHOLD * 2);

            Mat tempL(wholeSize, mImLeft.type()), masktempL;
            Mat tempR(wholeSize, mImRight.type()), masktempR;

            mPyramidLeft[level] = tempL(Rect(setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD, sz.width, sz.height));

            mPyramidRight[level] = tempR(Rect(setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if (level != 0) {
                resize(mPyramidLeft[level - 1], mPyramidLeft[level], sz, 0, 0, INTER_LINEAR);

                resize(mPyramidRight[level - 1], mPyramidRight[level], sz, 0, 0, INTER_LINEAR);

                copyMakeBorder(mPyramidLeft[level], tempL, setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD,
                               setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD,
                               BORDER_REFLECT_101 + BORDER_ISOLATED);

                copyMakeBorder(mPyramidRight[level], tempR, setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD,
                               setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD,
                               BORDER_REFLECT_101 + BORDER_ISOLATED);
            } else {
                copyMakeBorder(mImLeft, tempL, setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD,
                               setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD, BORDER_REFLECT_101);
                copyMakeBorder(mImRight, tempR, setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD,
                               setting::EDGE_THRESHOLD, setting::EDGE_THRESHOLD, BORDER_REFLECT_101);
            }
        }
        
    }

    bool Frame::PosInGrid(const shared_ptr<Feature> feature, int &posX, int &posY) {
        posX = int(feature->mPixel[0] * setting::GridElementWidthInv);
        posY = int(feature->mPixel[1] * setting::GridElementHeightInv);
        if (posX < 0 || posX >= setting::FRAME_GRID_COLS
            || posY < 0 || posY >= setting::FRAME_GRID_ROWS)
            return false;
        return true;
    }

    double Frame::ComputeSceneMedianDepth(const int &q) {
        unique_lock<mutex> lock(mMutexFeature);
        vector<double> vDepth;
        for (auto feat: mFeaturesLeft) {
            if (feat && feat->mfInvDepth > 0)
                vDepth.push_back(1.0 / feat->mfInvDepth);
        }

        if (vDepth.empty())
            return 0;

        sort(vDepth.begin(), vDepth.end());
        return vDepth[(vDepth.size() - 1) / q];
    }

    int Frame::TrackedMapPoints(const int &minObs) {
        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        unique_lock<mutex> lock(mMutexFeature);
        int N = mFeaturesLeft.size();
        for (int i = 0; i < N; i++) {
            if (mFeaturesLeft[i] == nullptr)
                continue;
            shared_ptr<MapPoint> pMP = mFeaturesLeft[i]->mpPoint;
            if (pMP) {
                if (!pMP->isBad()) {
                    if (bCheckObs) {
                        if (pMP->Observations() >= minObs)
                            nPoints++;
                    } else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }
    

    //search the first descriptor with descriptors and return the point of best match
    //and best match norm
    bool Frame::searchInArea(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
    {
        cv::Point2f best_pt;
        int bestDist = 128;
        int bestIndex = -1;
        
        //compare windows_descriptor with each one in the descriptors_old
        //and return distance
        for(int i = 0; i < descriptors_old.size(); i++)
        {
            int dis = HammingDis(window_descriptor, descriptors_old[i]);
            if(dis < bestDist)
            {
                bestDist = dis;
                bestIndex = i;
            }
        }

        //if the distance is small enough
        //return best match and best match norm
        if (bestIndex != -1 && bestDist < 80)
        {
            best_match = keypoints_old[bestIndex].pt;
            best_match_norm = keypoints_old_norm[bestIndex].pt;
            return true;
        }
        else
            return false;
    }
    
    
    void Frame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                                 std::vector<cv::Point2f> &matched_2d_old_norm,
                                 std::vector<uchar> &status,
                                 const std::vector<BRIEF::bitset> &descriptors_old,
                                 const std::vector<cv::KeyPoint> &keypoints_old,
                                 const std::vector<cv::KeyPoint> &keypoints_old_norm)
    {
        for(int i = 0; i < window_brief_descriptors.size(); i++)
        {
            cv::Point2f pt(0.f, 0.f);
            cv::Point2f pt_norm(0.f, 0.f);
            if (searchInArea(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
            {
                status.push_back(1);
            }
            else
            {
                status.push_back(0);
                matched_2d_old.push_back(pt);
                matched_2d_old_norm.push_back(pt_norm);
            }
        }
    }
    
    
    void Frame::computeWindowBRIEFPoint()
    {
        unique_lock<mutex> lock(mMutexFeature);
        //modify the path to the file
//         string BRIEF_PATTERN_FILE_PATH = "../../voc/brief_pattern.yml";
        string BRIEF_PATTERN_FILE_PATH = "/home/gishr/software/px4/slam_dev/x86_slam/voc/brief_pattern.yml";
        BriefExtractor extractor(BRIEF_PATTERN_FILE_PATH);
        for(int i = 0; i < point_2d_uv.size(); i++)
        {
            cv::KeyPoint key;
            if(mFeaturesLeft[i] == 0)
            {
                continue;
            }
            key.pt = point_2d_uv[i];
            window_keypoints.push_back(key);
        }
        
        extractor(mImLeft, window_keypoints, window_brief_descriptors);
    }
    
    
    void Frame::computeBRIEFPoint()
    {
        unique_lock<mutex> lock(mMutexFeature);
        unique_lock<mutex> lock2(mMutexGlobalKPs);
        //modify the path to the file
//         string BRIEF_PATTERN_FILE_PATH = "../../voc/brief_pattern.yml";
        string BRIEF_PATTERN_FILE_PATH = "/home/gishr/software/px4/slam_dev/x86_slam/voc/brief_pattern.yml";
        BriefExtractor extractor(BRIEF_PATTERN_FILE_PATH);

        for(int i = 0; i < mFeaturesLeft.size(); i++)
        {
            cv::KeyPoint key;
            if(mFeaturesLeft[i] == 0)
            {
                continue;
            }
            key.pt.x = mFeaturesLeft[i]->mPixel[0];
            key.pt.y = mFeaturesLeft[i]->mPixel[1];
            
            //NOTE put key to global keypoints
            global_keypoints.push_back(key);
            
            //NOTE set window_keypoints equal to global_keypoints
            window_keypoints.push_back(key);
            
            //NOTE also put keypoint to keypoint_old_norm
            keypoints_norm.push_back(key);
        }
       
        //NOTE forgive me, just do it now
        extractor(mImLeft, global_keypoints, brief_descriptors);
        extractor(mImLeft, window_keypoints, window_brief_descriptors);
        
    }
    
    
    int Frame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
    {
        BRIEF::bitset xor_of_bitset = a ^ b;
        int dis = xor_of_bitset.count();
        return dis;
    }
    
    
    template <typename Derived>
    void reduceVector(vector<Derived> &v, vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }
    
    
    vector<shared_ptr<MapPoint>> Frame::fetchMapPoints()
    {
        
        int featureVecSize = this->mFeaturesLeft.size();

        vector<shared_ptr<MapPoint>> tempVec;
        
        for(int i=0; i<featureVecSize; i++)
        {   
            
            shared_ptr<Feature>& feature = this->mFeaturesLeft[i];

            if(feature==nullptr || (feature->mpPoint)==nullptr || (feature->mpPoint)->isBad())
                continue;

            tempVec.push_back(feature->mpPoint);
        }
        
//         cout<<"Fetched MapPoint size before and after: "<<featureVecSize<<", "<<tempVec.size()<<endl;
        
        return tempVec;
    }
    
    vector<cv::Point3f> Frame::fetchMapPointsCV()
    {
        vector<shared_ptr<MapPoint> > fetchedMapPoints = fetchMapPoints();
                    
        vector<cv::Point3f> cvMapPoints = toCvPoint3f(fetchedMapPoints);
        
        return cvMapPoints;
    }
    
    
    vector<cv::Point2f> Frame::fetchFeaturesCV()
    {
        int featureVecSize = this->mFeaturesLeft.size();
        vector<cv::Point2f> tempKeyPointVec;
        vector<cv::Point3f> tempMapPointVec;
        
        cv::Point2f kp;
        cv::Point3f mp;
        
        for(int i=0; i<featureVecSize; i++)
        {
            shared_ptr<Feature>& feature = this->mFeaturesLeft[i];
            
            if(feature==nullptr || (feature->mpPoint)==nullptr || (feature->mpPoint)->isBad())
            {
                continue;
            }

            kp.x = feature->mPixel[0];
            kp.y = feature->mPixel[1];
            
            mp.x = (feature->mpPoint)->GetWorldPos()[0];
            mp.y = (feature->mpPoint)->GetWorldPos()[1];
            mp.z = (feature->mpPoint)->GetWorldPos()[2];
            
            tempMapPointVec.push_back(mp);
            tempKeyPointVec.push_back(kp);
            
        }
        
        return tempKeyPointVec;
    }
    
    
    void Frame::fetchKeyPointAndMapPoint(vector<cv::KeyPoint> CurrentKPs, vector<cv::Point3d> CurrentMPs)
    {
        
        cv::KeyPoint kp;
        cv::Point3d mp;
        
        int featureVecSize = this->mFeaturesLeft.size();
        
        for(int i=0; i<featureVecSize; i++)
        {
            shared_ptr<Feature>& feature = this->mFeaturesLeft[i];
            
            if(feature==nullptr || (feature->mpPoint)==nullptr || (feature->mpPoint)->isBad())
            {
                continue;
            }

            kp.pt.x = feature->mPixel[0];
            kp.pt.y = feature->mPixel[1];
            
            mp.x = (feature->mpPoint)->GetWorldPos()[0];
            mp.y = (feature->mpPoint)->GetWorldPos()[1];
            mp.z = (feature->mpPoint)->GetWorldPos()[2];
            
            CurrentKPs.push_back(kp);
            CurrentMPs.push_back(mp);
        }
        
        cout<<"fetchKeyPointAndMapPoint 222"<<CurrentKPs.size()<<", "<<CurrentMPs.size()<<endl;
        
    }
    
    
    std::tuple<vector<cv::KeyPoint>, vector<cv::Point3d> > Frame::fetchKeyPointAndMapPoint()
    {
        
        vector<cv::KeyPoint> CurrentKPs;
        vector<cv::Point3d> CurrentMPs;
        cv::KeyPoint kp;
        cv::Point3d mp;
        
        int featureVecSize = this->mFeaturesLeft.size();
        
        for(int i=0; i<featureVecSize; i++)
        {
            shared_ptr<Feature>& feature = this->mFeaturesLeft[i];
            
            if(feature==nullptr || (feature->mpPoint)==nullptr || (feature->mpPoint)->isBad())
            {
                continue;
            }

            kp.pt.x = feature->mPixel[0];
            kp.pt.y = feature->mPixel[1];
            
            mp.x = (feature->mpPoint)->GetWorldPos()[0];
            mp.y = (feature->mpPoint)->GetWorldPos()[1];
            mp.z = (feature->mpPoint)->GetWorldPos()[2];
            
            CurrentKPs.push_back(kp);
            CurrentMPs.push_back(mp);
        }
        
        std::tuple<vector<cv::KeyPoint>, vector<cv::Point3d> > result = std::make_tuple(CurrentKPs, CurrentMPs);
        cout<<"fetchKeyPointAndMapPoint 3 "<<CurrentKPs.size()<<", "<<CurrentMPs.size()<<endl;
        
        return result;
    }
    
    
    vector<cv::Point3f> Frame::toCvPoint3f(vector<shared_ptr<MapPoint>> mMapPoint)
    {
        
        vector<cv::Point3f> tempMapPoint;
        int size = mMapPoint.size();
        cv::Point3f cvPoint;
        for(int i=0; i<size; i++)
        {
            cvPoint.x = mMapPoint[i]->GetWorldPos()[0];
            cvPoint.y = mMapPoint[i]->GetWorldPos()[1];
            cvPoint.z = mMapPoint[i]->GetWorldPos()[2];
            
            tempMapPoint.push_back(cvPoint);
        }
        
        return tempMapPoint;
    }
    
    
    void Frame::updateFeatureAndMapPoints()
    {
        cout<<"this->mFeaturesLeft.size() size before: "<<this->mFeaturesLeft.size()<<endl;
        
        int featureVecSize = this->mFeaturesLeft.size();

        for(int i=0; i<featureVecSize; i++)
        {
            shared_ptr<Feature>& feature = this->mFeaturesLeft[i];
            vector<shared_ptr<Feature> >::iterator iter;
            
            for(iter=this->mFeaturesLeft.begin(); iter!=this->mFeaturesLeft.end(); iter ++)
            {
                if(*iter==nullptr || ((*iter)->mpPoint)==nullptr || ((*iter)->mpPoint)->isBad())
                    this->mFeaturesLeft.erase(iter);
            }
        }
        
        cout<<"this->mFeaturesLeft.size() size after: "<<this->mFeaturesLeft.size()<<endl;
        
    }
    
    
    //today's work
    bool Frame::findConnection(shared_ptr<Frame> pFrame)
    {
        unique_lock<mutex> lock(mMutexGlobalKPs);
        vector<cv::Point2f> matched_2d_cur, matched_2d_old;
        vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
        vector<cv::Point3f> matched_3d;
        vector<double> matched_id;
        vector<uchar> status;
        
        
        // in VINS, point3d is defined as (x, y, 1)
        matched_3d = point_3d;
        matched_2d_cur = point_2d_uv;
        matched_2d_cur_norm = point_2d_norm;
        matched_id = point_id;
        
        cout<<"Frame::findConnection 1"<<endl;
        
        searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, pFrame->brief_descriptors, pFrame->global_keypoints, pFrame->keypoints_norm);
        reduceVector(matched_2d_cur, status);
        reduceVector(matched_2d_old, status);
        reduceVector(matched_2d_cur_norm, status);
        reduceVector(matched_2d_old_norm, status);
        reduceVector(matched_3d, status);
        reduceVector(matched_id, status);
        status.clear();
        
        cout<<"Frame::findConnection 2"<<endl;
        
        Eigen::Vector3d PnP_T_old;
        Eigen::Matrix3d PnP_R_old;
        Eigen::Vector3d relative_t;
        Quaterniond     relative_q;
        double          relative_yaw;
        
        cout<<"Frame::findConnection 3"<<endl;
        
        if(matched_2d_cur.size() > 25)
        {
            status.clear();
            
            //given 3d points and 2d projected points, find T and R
            PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
            reduceVector(matched_2d_cur, status);
            reduceVector(matched_2d_old, status);
            reduceVector(matched_2d_cur_norm, status);
            reduceVector(matched_2d_old_norm, status);
            reduceVector(matched_3d, status);
            reduceVector(matched_id, status);
        }
        
        cout<<"Frame::findConnection 4"<<endl;
        
        if (matched_2d_cur.size() > 25)
        {
            relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
            relative_q = PnP_R_old.transpose() * origin_vio_R;
            relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());
            
            cout<<"Frame::findConnection 5"<<endl;
            if (abs(relative_yaw)<30.0 && relative_t.norm()<20.0)
            {
                has_loop = true;
                loop_index = pFrame->index;
                loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                             relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                             relative_yaw;
                             
                cout<<"Frame::findConnection 6"<<endl;
                             
                //TODO modify this part
                bool FAST_RELOCALIZATION = 1;
//                 if(FAST_RELOCALIZATION)
//                 {
//                     sensor_msgs::PointCloud msg_match_points;
//                     msg_match_points.header.stamp = ros::Time(time_stamp);
//                     for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
//                     {
//                         geometry_msgs::Point32 p;
//                         p.x = matched_2d_old_norm[i].x;
//                         p.y = matched_2d_old_norm[i].y;
//                         p.z = matched_id[i];
//                         msg_match_points.points.push_back(p);
//                     }
//                     Eigen::Vector3d T = old_kf->T_w_i; 
//                     Eigen::Matrix3d R = old_kf->R_w_i;
//                     Quaterniond Q(R);
//                     sensor_msgs::ChannelFloat32 t_q_index;
//                     t_q_index.values.push_back(T.x());
//                     t_q_index.values.push_back(T.y());
//                     t_q_index.values.push_back(T.z());
//                     t_q_index.values.push_back(Q.w());
//                     t_q_index.values.push_back(Q.x());
//                     t_q_index.values.push_back(Q.y());
//                     t_q_index.values.push_back(Q.z());
//                     t_q_index.values.push_back(index);
//                     msg_match_points.channels.push_back(t_q_index);
//                     pub_match_points.publish(msg_match_points);
//                 }
                
                return true;
            }
            cout<<"Frame::findConnection 7"<<endl;
            
        }
        return false;
        
    }
    
    //oh my beloved PNP and RANSAC
    void Frame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
    {
        // find out what R initial and T initial means
        cv::Mat r, rvec, t, D, tmp_r;
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
        Matrix3d R_inital;
        Vector3d P_inital;
        
        //TODO modify this part
        //origin_vio_R = vio_R_w_i
        
        //Matrix3d R_w_c = origin_vio_R * qic;
        //Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;
        
        Matrix3d R_w_c = Rwb();
        Vector3d T_w_c = Twb();
        
        R_inital = R_w_c.inverse();
        P_inital = -(R_inital * T_w_c);
        
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        
        cv::Mat inliers;
        
        if (CV_MAJOR_VERSION < 3)
            cv::solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
        else
        {
            if (CV_MINOR_VERSION < 2)
                cv::solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
            else
                cv::solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);
        }
        
        for (int i = 0; i < matched_2d_old_norm.size(); i++)
        {
            status.push_back(0);
        }
        
        for( int i = 0; i < inliers.rows; i++)
        {
            int n = inliers.at<int>(i);
            status[n] = 1;
        }
        
        cv::Rodrigues(rvec, r);
        Matrix3d R_pnp, R_w_c_old;
        cv::cv2eigen(r, R_pnp);
        R_w_c_old = R_pnp.transpose();
        Vector3d T_pnp, T_w_c_old;
        cv::cv2eigen(t, T_pnp);
        T_w_c_old = R_w_c_old * (-T_pnp);
        
        //TODO rethink this part
        //PnP_R_old = R_w_c_old * qic.transpose();
        //PnP_T_old = T_w_c_old - PnP_R_old * tic;
        
        //
        PnP_R_old = R_w_c_old;
        PnP_T_old = T_w_c_old;
        
    }
    
    
    Eigen::Vector3d Frame::getLoopRelativeT()
    {
        return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
    }
   
    Eigen::Quaterniond Frame::getLoopRelativeQ()
    {
        return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
    }
    
    double Frame::getLoopRelativeYaw()
    {
        return loop_info(7);
    }
    
    void Frame::processPoints()
    {
        for (auto p : mFeaturesLeft)
        {
            if(p->mpPoint != nullptr)
                continue;
            
            cv::Point3f p3f;
            p3f.x = (p->mpPoint)->mWorldPos[0];
            p3f.y = (p->mpPoint)->mWorldPos[1];
            p3f.z = (p->mpPoint)->mWorldPos[2];
            point_3d.push_back(p3f);
            
            cv::Point2f p2f;
            p2f.x = p->mPixel[0];
            p2f.y = p->mPixel[1];
            point_2d_uv.push_back(p2f);
        }
        
        computeBRIEFPoint();
    }

} //namespace ygz


