#include "ygz/TrackerLK.h"
#include "ygz/ORBExtractor.h"
#include "ygz/ORBMatcher.h"
#include "ygz/LKFlow.h"
#include "ygz/Feature.h"
#include "ygz/BackendInterface.h"
#include "ygz/MapPoint.h"
#include "ygz/Viewer.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

namespace ygz {

    TrackerLK::TrackerLK(const string &settingFile) {
        LOG(WARNING)<<"Init Octree."<<endl;
        this->frame_to_build_count_mutex.lock();
        this->frame_to_build_count = 0;
        this->frame_id = 0;
        this->frame_to_build_count_mutex.unlock();


        cv::FileStorage fSettings(settingFile, cv::FileStorage::READ);
        if (fSettings.isOpened() == false) {
            cerr << "Setting file not found." << endl;
            return;
        }

        // create camera object
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];
        float bf = fSettings["Camera.bf"];

        mpCam = shared_ptr<CameraParam>(new CameraParam(fx, fy, cx, cy, bf));
        mpExtractor = shared_ptr<ORBExtractor>(new ORBExtractor(ORBExtractor::FAST_SINGLE_LEVEL));
        //mpExtractor = shared_ptr<ORBExtractor>(new ORBExtractor(ORBExtractor::ORB_SLAM2));
        mpMatcher = shared_ptr<ORBMatcher>(new ORBMatcher);
        
        mpMapSerialization = shared_ptr<MapSerialization>(new MapSerialization());
        
        mState = NO_IMAGES_YET;
        
        //this->mBuildObstacleMap -> false
        if(false)
        {
            //mtBackendMainLoop = thread(&BackendSlidingWindowG2O::MainLoop, this);
            this->obstacle_map_thread = thread(&Tracker::do_obstacle_map_building_main,this);
            this->obstacle_map_thread.detach();//分离出来独立运行即可.
        }
    }

    TrackerLK::TrackerLK() {
        mState = NO_IMAGES_YET;
    }

    
    SE3d TrackerLK::InsertStereo(            
	    const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
        const VecIMU &vimu,
        VehicleAttitude va,
        bool use_atti,
        double gps_x, double gps_y, double gps_z, bool use_gps, double height, bool use_height)
    {
        
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        if (setting::trackerUseHistBalance)
        {
            // perform a histogram equalization
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            Mat imgLeftAfter, imgRightAfter;
            clahe->apply(imRectLeft, imgLeftAfter);
            clahe->apply(imRectRight, imgRightAfter);
            mpCurrentFrame = shared_ptr<Frame>(new Frame(imgLeftAfter, imgRightAfter, timestamp, mpCam, vimu));
        }
        else
        {
            mpCurrentFrame = shared_ptr<Frame>(new Frame(imRectLeft, imRectRight, timestamp, mpCam, vimu));
        }
        
        shared_ptr<FrameLight> pFrameLight = make_shared<FrameLight>(this->frame_id);
        this->id_to_frame_map[this->frame_id] = pFrameLight;
	
        if (this->mbVisionOnlyMode == false&&vimu.size()>0)
        {
            mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());   // 追加imu数据
        }

        if (mpLastKeyFrame)
        {
            mpCurrentFrame->mpReferenceKF = mpLastKeyFrame;
        }

        // DO TRACKING !!
        LOG(INFO) << "\n\n********* Tracking frame " << mpCurrentFrame->mnId << " **********" << endl;
        this->mGPSx = gps_x;
        this->mGPSy = gps_y;
        this->mGPSz = gps_z;
        this->mUseGPS = use_gps;

        this->mVA = va;
        this->use_mVA =  use_atti;
        this->mHeight = height;
        this->use_mHeight = use_height;


        LOG(INFO)<<"Tracking frame 1"<<endl;

        Track();

        LOG(INFO)<<"Tracking frame 2"<<endl;

        //NOTE for serialization
//        if (mpCurrentFrame->IsKeyFrame() && mpMapSerialization!=nullptr)
//        {
//            mpMapSerialization->addFrame(mpCurrentFrame);
//
//            if(mpMapSerialization->getSize()>200)
//            {
//                string path = "./MapSerialization.bin";
//                mpMapSerialization->serialize(path);
//                mpMapSerialization->test();
//            }
//        }
        
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();


        if (mpViewer)
        {
            if (mpCurrentFrame->IsKeyFrame())
            {
                mpViewer->AddFrame(mpCurrentFrame);
            }
            else
            {
                mpViewer->SetCurrentFrame(mpCurrentFrame);
            }
            mpViewer->SetTrackStatus(static_cast<int>(mState), mTrackInliersCnt);
        }


        LOG(INFO)<<"Tracking frame 3"<<endl;

        auto position_var = mpCurrentFrame->GetPose().translation();

        
        LOG(INFO)<<"Tracker position = "<<position_var[0]<<","<<position_var[1]<<","<<position_var[2]<<","<<std::setprecision(100)<<endl;
        if (mbVisionOnlyMode == false)
        {
            LOG(INFO) << "speed and bias = \n" << mpCurrentFrame->mSpeedAndBias.transpose() << endl;
        }
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();

        
        //this->id_to_frame_map[this->frame_id++] = pNewFrame;//OK
        pFrameLight->sync_with_frame(*mpCurrentFrame);
        //shared_ptr<FrameLight> pCopy(pFrameLight);//make a copy.

        
        this->frame_to_build_count_mutex.lock();
        this->frame_to_build_count+=1;
        this->frame_id+=1;
        this->frame_to_build_count_mutex.unlock();

        return mpCurrentFrame->GetPose();

    }

    void TrackerLK::Track() {

        mTrackInliersCnt = 0;

        if (mState == NO_IMAGES_YET) {

            // 尝试通过视觉双目构建初始地图
            // first we build the pyramid and compute the features
            LOG(INFO) << "Detecting features" << endl;
            mpExtractor->Detect(mpCurrentFrame, true, false);    // extract the keypoint in left eye
            LOG(INFO) << "Compute stereo matches" << endl;
            mpMatcher->ComputeStereoMatches(mpCurrentFrame, ORBMatcher::OPTIFLOW_CV);

            if (StereoInitialization() == false) {
                LOG(INFO) << "Stereo init failed." << endl;
                return;
            }

            // 向后端添加关键帧
            InsertKeyFrame();

            mpLastFrame = mpCurrentFrame;
            mpLastKeyFrame = mpCurrentFrame;

            // 置状态为等待初始化状态
            mState = NOT_INITIALIZED;
            return;

        }
        else if (mState == NOT_INITIALIZED) 
        {   //不使用imu会永远停留在这个状态。
            // IMU未初始化时，位姿预测不靠谱，先用纯视觉追踪
            bool bOK = false;
            mpCurrentFrame->SetPose(mpLastFrame->GetPose() * mSpeed);  // assume the speed is constant
            bOK = TrackLastFrame(false);
            if (bOK)
            {
                bOK = TrackLocalMap(mTrackInliersCnt);
            }
            if (bOK == false)
            {
                // 纯视觉追踪失败了，reset整个系统
                // 我们并不希望出现这个结果
                LOG(WARNING) << "Pure vision tracking failed! Reseting system!!" << endl;
                Reset();
                return;
            }
            
            CleanOldFeatures();

            if (NeedNewKeyFrame(mTrackInliersCnt))
            {
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                CreateStereoMapPoints();

                InsertKeyFrame();

                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                LOG(INFO) << "Insert KF cost time: " << timeCost << endl;
            }
            else
            {

            }

            // 尝试带着IMU初始化
            if (mbVisionOnlyMode == false)
            {
                if (IMUInitialization() == true)
                {
                    // 初始化成功，置状态为OK
                    mState = OK;
                }
            }

            mSpeed = mpLastFrame->GetPose().inverse() * mpCurrentFrame->GetPose();
            mpLastFrame = mpCurrentFrame;
	    
            if(this->mBuildObstacleMap)
            {
                //this->mCloud;//TODO:do something to insert into global temporary cloud or octree.
            }


            return;

        }
        else if (mState == OK) {
            // 正常追踪
            // 用imu预测位姿，再尝试用预测的位姿进行特征匹配
            bool bOK = false;

            /*
            if (rand() % 10 == 0) {
                // 故意设置丢失
                LOG(INFO) << "Set to lost!" << endl;
                mpCurrentFrame->mImLeft.setTo(0);
                mpCurrentFrame->mImRight.setTo(0);
            }
             */

            PredictCurrentPose();
            // 与上一个帧比较
            bOK = TrackLastFrame(false);

            if (bOK) {
                // 与lastframe比对之后，current有了粗略的pose估计，利用此信息与local map进行对比
                bOK = TrackLocalMap(mTrackInliersCnt);
            }

            if (bOK) {
                // 流程成功
                CleanOldFeatures();

                if (NeedNewKeyFrame(mTrackInliersCnt)) {
                    // 处理关键帧
                    CreateStereoMapPoints();
                    InsertKeyFrame();
                }

                mpLastFrame = mpCurrentFrame;
                mSpeed = mpLastFrame->GetPose().inverse() * mpCurrentFrame->GetPose();    // 设置速度

            } else {
                mState = WEAK;
                mpLastFrame = mpCurrentFrame;
                LOG(INFO) << "Set into WEAK mode" << endl;
                // in this case, don't save current as last frame
            }
        } else if (mState == WEAK) {
            LOG(INFO) <<"track 7"<<endl;
            LOG(WARNING) << "Running WEAK mode" << endl;
            // use imu only to propagate the pose and try to initialize stereo vision
            PredictCurrentPose();   // 这步可能不可靠

            // first let's try to track the last (may be bit longer) frame
            bool bOK = false;
            bOK = TrackLastFrame(false);
            bOK = TrackLocalMap(mTrackInliersCnt);
            if (bOK) {
                // we successfully tracked the last frame and local map, set state back to OK
                mState = OK;
                // 处理关键帧
                if (NeedNewKeyFrame(mTrackInliersCnt))
                    InsertKeyFrame();
                mpLastFrame = mpCurrentFrame;
            } else {
                // track failed, try use current frame to init the stereo vision
                // grab the features and do stereo matching
                mpExtractor->Detect(mpCurrentFrame, true, false);
                mpMatcher->ComputeStereoMatches(mpCurrentFrame, ORBMatcher::OPTIFLOW_CV);
                mpBackEnd->Reset();
                CreateStereoMapPoints();
                CleanOldFeatures();
                InsertKeyFrame();
                mState = OK;
                mpLastFrame = mpCurrentFrame;
                LOG(INFO) << "Recovered from WEAK into NOT_INITIALIZAED." << endl;
            }
        }
    }

    bool TrackerLK::TrackLastFrame(bool usePoseInfo) {

        // Track the points in last frame and create new features in current
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        VecVector2f trackedPts, refPts;
        vector<size_t> idxRef;
        SE3d TCW = mpCurrentFrame->GetTCW();
        unique_lock<mutex> lock(mpLastFrame->mMutexFeature);


        for (size_t i = 0; i < mpLastFrame->mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> feat = mpLastFrame->mFeaturesLeft[i];
            if (feat == nullptr) {
                continue;
            }

            idxRef.push_back(i);
            if (feat->mpPoint && feat->mpPoint->isBad() == false) {
                // associated with a good map point, predict the projected pixel in current
                refPts.push_back(feat->mPixel);
                // trackedPts.push_back(feat->mPixel);
                Vector2d px = mpCurrentFrame->World2Pixel(feat->mpPoint->GetWorldPos(), TCW);
                trackedPts.push_back(Vector2f(px[0], px[1]));
            } else {
                refPts.push_back(feat->mPixel);
                trackedPts.push_back(feat->mPixel);
            }
        }


        // 注意 LK 只管追踪2D点，而不管这些2D点是否关联了3D地图点
        int cntMatches = LKFlowCV(mpLastFrame, mpCurrentFrame, refPts, trackedPts);
        // int cntMatches = LKFlow(mpLastFrame, mpCurrentFrame, trackedPts);

        int validMatches = 0;


        for (size_t i = 0; i < trackedPts.size(); i++) {
            if (trackedPts[i][0] < 0 || trackedPts[i][1] < 0)
                continue;

            // create a feature assigned with this map point
            shared_ptr<Feature> feat(new Feature);
            feat->mPixel = trackedPts[i];
            feat->mpPoint = mpLastFrame->mFeaturesLeft[idxRef[i]]->mpPoint;// a 3d point in global map in last frame.
            mpCurrentFrame->mFeaturesLeft.push_back(feat);
            if (feat->mpPoint->Status() == MapPoint::GOOD) {
                validMatches++;
            }

        }


        //LOG(INFO) << "Current features: " << mpCurrentFrame->mFeaturesLeft.size() << endl;

        if (validMatches <= setting::minTrackLastFrameFeatures) 
        {//failed!
            LOG(WARNING) << "Track last frame not enough valid matches: " << validMatches << ", I will abort this frame"
                         << endl;
            return false;
        }

        // match last frame successfully ,do pose optimization.
        //LOG(INFO) << "      LK tracked points: " << cntMatches << ", valid: " << validMatches << ", last frame features: "<< mpLastFrame->mFeaturesLeft.size() << endl;

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        //LOG(INFO) << "      LK in last frame cost time: " << timeCost << ", pts: " << refPts.size() << endl;


        try {
            mTrackInliersCnt = OptimizeCurrentPose();
            // LOG(INFO) << "Track last frame inliers: " << inliers << endl;
        }
        catch (exception& e)
        {
            cout << "Standard exception: " << e.what() << endl;
        }


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t3).count();
        //LOG(INFO) << "      Optimization in last frame time cost:"<<timeCost <<endl;
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //LOG(INFO) << "Track last frame total cost time: " << timeCost << endl;

        if (mTrackInliersCnt >= setting::minTrackLastFrameFeatures) {
            return true;
        } else {
            return false;
        }
    }

    bool TrackerLK::TrackLocalMap(int &inliers) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Step 1. 从Local Map中投影到当前帧
        set<shared_ptr<MapPoint>> localmap = mpBackEnd->GetLocalMap();

        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t1).count();
        //LOG(INFO) << "      get local map points cost time: " << timeCost << endl;

        for (auto mp: localmap)
            if (mp)
                mp->mbTrackInView = false;

        for (auto feat: mpCurrentFrame->mFeaturesLeft)
            if (feat && feat->mpPoint && feat->mbOutlier == false)
                feat->mpPoint->mbTrackInView = true;

        // 筛一下视野内的点
        set<shared_ptr<MapPoint> > mpsInView;
        for (auto &mp: localmap) {
            if (mp && mp->isBad() == false && mp->mbTrackInView == false && mpCurrentFrame->isInFrustum(mp, 0.5)) {
                mpsInView.insert(mp);
            }
        }

        if (mpsInView.empty())
            return inliers >= setting::minTrackLocalMapInliers;

        //LOG(INFO) << "Call Search by direct projection" << endl;
        int cntMatches = mpMatcher->SearchByDirectProjection(mpCurrentFrame, mpsInView);
        //LOG(INFO) << "Track local map matches: " << cntMatches << ", current features: "<< mpCurrentFrame->mFeaturesLeft.size() << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t4).count();
        //LOG(INFO) << "      Filter map point and search local map points cost time: " << timeCost << endl;

        // Optimize Pose
        int optinliers = OptimizeCurrentPose();

        // Update MapPoints Statistics
        inliers = 0;
        for (shared_ptr<Feature> feat : mpCurrentFrame->mFeaturesLeft) {
            if (feat->mpPoint) {
                if (!feat->mbOutlier) {
                    feat->mpPoint->IncreaseFound();
                    if (feat->mpPoint->Status() == MapPoint::GOOD)
                        inliers++;
                } else {
                }
            }
        }

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
        //LOG(INFO) << "      Optimization in local map cost time: " << timeCost << endl;
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        //LOG(INFO) << "Track locak map cost time:"<<timeCost<<endl;
        //LOG(INFO) << "Track Local map inliers: " << inliers << endl;

        // Decide if the tracking was succesful
        if (inliers < setting::minTrackLocalMapInliers)
            return false;
        else
            return true;
    }

    void TrackerLK::Reset() {

        LOG(INFO) << "Tracker is reseted" << endl;
        mpCurrentFrame->SetPose(mpLastFrame->GetPose());
        LOG(INFO) << "Current pose = \n" << mpCurrentFrame->GetPose().matrix() << endl;
        mpBackEnd->Reset();

        // test if we can just recover from stereo
        mpCurrentFrame->mFeaturesLeft.clear();

        LOG(INFO) << "Try init stereo" << endl;
        mpExtractor->Detect(mpCurrentFrame, true, false);    // extract the keypoint in left eye
        mpMatcher->ComputeStereoMatches(mpCurrentFrame, ORBMatcher::OPTIFLOW_BASED);

        if (StereoInitialization() == false) {
            LOG(INFO) << "Stereo init failed." << endl;
            // 置状态为等待初始化状态
            mState = NOT_INITIALIZED;
            return;
        } else {
            LOG(INFO) << "Stereo init succeed." << endl;
            // set the current as a new kf and track it
            InsertKeyFrame();

            mpLastFrame = mpCurrentFrame;
            mpLastKeyFrame = mpCurrentFrame;
        }
        return;
    }

    void TrackerLK::CreateStereoMapPoints() {

        // 尝试通过左右双目建立一些地图点
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        mpCurrentFrame->AssignFeaturesToGrid();
        mpExtractor->Detect(mpCurrentFrame, true, false);    // extract new keypoints in left eye

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        double timeCost2 = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        LOG(INFO) << "Detect new feature cost time: " << timeCost2 << endl;

        // 对未形成地图点的部分进行左右目匹配
        mpMatcher->ComputeStereoMatchesOptiFlow(mpCurrentFrame, true);

        float meanInvDepth = 1.0 / (mpCurrentFrame->ComputeSceneMedianDepth(2) + 1e-9);

        int cntMono = 0, cntStereo = 0, cntUpdate = 0;
        // 处理匹配结果
        for (size_t i = 0; i < mpCurrentFrame->mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> feat = mpCurrentFrame->mFeaturesLeft[i];
            if (feat == nullptr)
	    {
                continue;
	    }
            if (feat->mpPoint == nullptr) 
	    {
                // 来自新的特征点
                if (feat->mfInvDepth > setting::minNewMapPointInvD && feat->mfInvDepth < setting::maxNewMapPointInvD) {
                    // 双目匹配成功
                    // we create a map point here
                    shared_ptr<MapPoint> mp(new MapPoint(mpCurrentFrame, i));   // 创建新地图点
                    feat->mpPoint = mp;
		    
		    //build obstacle_map
		    if(this->mBuildObstacleMap)
		    {
		        shared_ptr<FrameLight> pFrameLight = this->id_to_frame_map[this->frame_id];
		        mpCurrentFrame->obs_lock.lock();
			    PointerObs pObs = make_shared<ObstacleCandidate>(
				mpCurrentFrame->Rwc().inverse() * (mp->GetWorldPos() - mpCurrentFrame->Ow()),feat
					  );
		        pFrameLight->obstacle_list.push_back(pObs); // the 3d pos is relative to its owner KF.
			mpCurrentFrame->obs_lock.unlock();
		    }
                    cntStereo++;
                } else {
                    //cout<<"Dis failed.distance:"<<1.0/feat->mfInvDepth<<endl;
                    // 双目匹配不成功，增加单目特征点
                    feat->mfInvDepth = meanInvDepth;
                    shared_ptr<MapPoint> mp(new MapPoint(mpCurrentFrame, i));   // 创建新地图点
                    feat->mpPoint = mp;
                    mp->SetStatus(MapPoint::IMMATURE);
                    cntMono++;
                }
            } 
            else 
	    {
                // 已经关联了地图点
                if (feat->mpPoint->Status() == MapPoint::IMMATURE) {
                    if (feat->mpPoint->mpRefKF.expired() ||
                        (feat->mpPoint->mpRefKF.expired() == false &&
                         feat->mpPoint->mpRefKF.lock()->mbIsKeyFrame == false))
                        feat->mpPoint->mpRefKF = mpCurrentFrame;    // change its reference

                    if (feat->mfInvDepth > setting::minNewMapPointInvD &&
                        feat->mfInvDepth < setting::maxNewMapPointInvD) {
                        // 逆深度有效，说明在左右目之间匹配到了未成熟的地图点
                        feat->mpPoint->SetStatus(MapPoint::GOOD);
                        Vector3d ptFrame =
                                mpCurrentFrame->mpCam->Img2Cam(feat->mPixel) * (1.0 / double(feat->mfInvDepth));
                        feat->mpPoint->SetWorldPos(mpCurrentFrame->mRwc * ptFrame + mpCurrentFrame->mOw);
                        cntUpdate++;
                    } else {
                        // 如果没有，那么将invDepth设成根据地图点世界坐标推算出来的，并保持这个点为IMMATURE
                        Vector3d pw = feat->mpPoint->mWorldPos;
                        feat->mfInvDepth = 1.0 / (mpCurrentFrame->mRcw * pw + mpCurrentFrame->mtcw)[2];
                    }
                }
            }
        }

        LOG(INFO) << "new stereo: " << cntStereo << ", new Mono: " << cntMono << ", update immature: " << cntUpdate
                  << ", total features: " << mpCurrentFrame->mFeaturesLeft.size() << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        LOG(INFO) << "Create map point cost time: " << timeCost << endl;
    }

    void TrackerLK::CleanOldFeatures() {

        //LOG(INFO) << "Cleaning old features" << endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        int cntPassed = 0;
        int cntOutlier = 0;
        for (shared_ptr<Feature> &feat: mpCurrentFrame->mFeaturesLeft) {
            if (feat->mbOutlier) {
                feat = nullptr; // 取消这个特征
                // 但是由于特征是由vector结构存储的，不便删去，因此保留一个空指针
                cntOutlier++;
                continue;
            }

            shared_ptr<MapPoint> mp = feat->mpPoint;
            auto status = mp->Status();
            if (status == MapPoint::BAD) {
                // 不保留对坏地图点的观测
                feat->mpPoint = nullptr;
                feat = nullptr;
            } else if (status == MapPoint::GOOD) {
                // 好的地图点
                if (mp->mpRefKF.expired() || mp->mpRefKF.lock()->IsKeyFrame() == false) {
                    // 该地图点由非关键帧提取
                    mp->mpRefKF = mpCurrentFrame; // 转到自身
                    cntPassed++;
                }
            } else {
                // 未成熟的地图点
                if (mp->mpRefKF.expired() || mp->mpRefKF.lock()->IsKeyFrame() == false) {
                    // 该地图点由非关键帧提取
                    mp->mpRefKF = mpCurrentFrame; // 转到自身
                    cntPassed++;
                }
            }
        }
        //LOG(INFO) << "passed " << cntPassed << " features into current, delete "<< cntOutlier << " outliers." << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        //LOG(INFO) << "Clean old features cost time: " << timeCost << endl;

    }
}
