#include "controller.h"


Controller::Controller(ros::NodeHandle& nh)
{
    mNH = nh;
    //mMavrosSub = mNH.subscribe("/mavros/vision_pose/pose", 100, &Controller::MavrosPoseCallback, this);
    //mMavrosSub = mNH.subscribe("/mavros/local_position/pose", 100, &Controller::MavrosPoseCallback, this);
    mTargetSetSub = mNH.subscribe("/move_base_simple/goal", 100, &Controller::TargetSetSubCallback, this);

    mPositionControlPub = mNH.advertise<geometry_msgs::PoseStamped>("gi/set_pose/position", 100);
    mYawControlPub = mNH.advertise<std_msgs::Float32>("gi/set_pose/orientation", 100);

    mSceneRetrievedPosition = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
    mTargetPose.pose.position.x = 0;
    mTargetPose.pose.position.y = 0;
    mTargetPose.pose.position.z = 0;


    mCONTROLLERSTATE = mControllerState ::NO_SCENE_RETRIEVED_BEFORE;
    mTARGET = mTarget::NO_TARGET;

    this->PosePublisher = this->nh.advertise<visualization_msgs::Marker>("/updated_target",10);

    std::thread t(&Controller::Run, this);
    t.detach();

//    mpWorld = make_shared<World>();

    testfile.open("all.txt", fstream::out | fstream::app);
}

void Controller::Run()
{
    ros::Rate rate(10.0);
    while (!ros::isShuttingDown())
    {
        //constantly fetching information from MAVROS
//        if(mCONTROLLERSTATE == MAVROS_STATE_ERROR)
//        {
//            Hover();
//        }

        if(mTARGET == NO_TARGET)
        {
            continue;
        }
        else if (mTARGET == NEW_TARGET)
        {
//            int building_idx = -1;
//            bool result = mpWorld->isInBuilding(mTargetPose, building_idx, true);
//            cout<<"Is mTargetPose in building?: "<<result<<endl;
//
//            auto wps = mpWorld->FindWayPoints(mMavrosPose11, mTargetPose);
//            for(auto& wp : wps)
//            {
//                LOG(INFO)<<"Current waypoints: "<<wp<<endl;
//            }
//
//            GoByWayPoints(wps);
//            //GoToTarget(mTargetPose);
//
//            // disable movement after finishing current task mission.
//            mTARGET = NO_TARGET;
        }

        rate.sleep();
    }

    Hover();
}

void Controller::SetTarget(geometry_msgs::PoseStamped& target)
{
    mTargetPose = target;
    mTARGET = NEW_TARGET;
}


void Controller::GoByWayPoints(vector<geometry_msgs::PoseStamped>& way_points)
{
    if(way_points.empty())
        LOG(INFO)<<"Way points empty!"<<endl;

    for(auto& wp : way_points)
    {
        LOG(INFO)<<"Current waypoint: "<<wp<<endl;
        GoToTarget(wp);
        ros::Duration(5).sleep();
    }
}

bool Controller::GoToTarget(const geometry_msgs::PoseStamped& target, bool useBodyFrame)
{
    publishPose(target);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();

    //ROS uses ENU internally, so we will stick to this convention

    //NOTE the callback from RVIZ is in FLU frame, to convert it to ENU frame we have to
    //apply a custom transformation
    if (useBodyFrame)
        pose.header.frame_id = "base_link";
    else {
        pose.header.frame_id = "map";

        pose.pose.position.x =  - target.pose.position.y;
        pose.pose.position.y = target.pose.position.x;
        pose.pose.position.z = target.pose.position.z;
    }

    LOG(INFO)<<"Going to target: "<<pose.pose.position.x<<", "
                                  <<pose.pose.position.y<<", "
                                  <<pose.pose.position.z<<endl;

    mPositionControlPub.publish(pose);
}

void Controller::AddRetrievedPose(cv::Mat retrieved_pose, cv::Mat mavros_pose)
{

//    if (!mMavrosInitialized)
//        return;

//    std::lock_guard<std::mutex> lock(mMavrosMutex);

    deque<cv::Mat> mRetrievedPoseQueue11;

    if(mavros_pose.empty() || retrieved_pose.empty())
        return;

//    LOG(INFO)<<"AddRetrievedPose start. 1"<<endl;
//
//    MatToPoseStamped(mavros_pose, "map");
//
//    LOG(INFO)<<"AddRetrievedPose start. 0.5"<<endl;
//
//    mMavrosPose11 = MatToPoseStamped(mavros_pose, "map");

    geometry_msgs::PoseStamped mMavrosPose11;
    mMavrosPose11.pose.position.x = 0;
    mMavrosPose11.pose.position.y = 0;
    mMavrosPose11.pose.position.z = 0;

//    LOG(INFO)<<"AddRetrievedPose start. 1"<<endl;
//
//    cv::Mat current_drone_pose_mat = PoseStampedToMat(mMavrosPose11);
//
//    LOG(INFO)<<"AddRetrievedPose start."<<endl;

    if (retrieved_pose.empty() || mavros_pose.empty())
        return;

    LOG(INFO)<<"AddRetrievedPose, pose not empty."<<endl;

    // outlier determination
    LOG(INFO)<<"AddRetrievedPose, pose distance."<<endl;
    float relative_distance = PoseDistance(retrieved_pose, mavros_pose);
    LOG(INFO)<<"AddRetrievedPose, pose distance.: "<<relative_distance<<endl;

//    if(mSceneMavrosDistances.empty())
//        mSceneMavrosDistances.push_back(relative_distance);

    LOG(INFO)<<"AddRetrievedPose, before outlier determintation"<<endl;
//    if(isOutlier(retrieved_pose, mavros_pose))
//        return;
    LOG(INFO)<<"AddRetrievedPose, current retrieved pose not outlier"<<endl;

//    mSceneMavrosDistances.push_back(relative_distance);
    LOG(INFO)<<"AddRetrievedPose 2"<<endl;
    // use mavros pose when starting to retrieve pose
    // you can use debug_scripts/debug.py to visualize mavros pose and retrieved
    // pose.
//    string frame = "map";
//    mMavrosPose11 = MatToPoseStamped(mavros_pose, frame);
    LOG(INFO)<<"AddRetrievedPose 3"<<endl;
//    mSceneRetrievedLastPosition = mSceneRetrievedPosition;
//    mSceneRetrievedPosition = retrieved_pose;
//
//    mMavPoseLastRetrieved = mMavPoseCurRetrieved;
//    mMavPoseCurRetrieved = mMavrosPose11;
   LOG(INFO)<<"AddRetrievedPose 4"<<endl;
//    if (abs(mMavrosPose11.pose.position.x)+abs(mMavrosPose11.pose.position.y)+abs(mMavrosPose11.pose.position.z)==0)
//    {
//        LOG(WARNING)<<"Current Mavros Pose not valid, adding retrieved pose failed!"<<endl;
//        return;
//    }

  mControllerState mCONTROLLERSTATE11 = NO_SCENE_RETRIEVED_BEFORE;

    // the first retrieved pose
    // NOTE assuming the first retrieved pose is "right" TODO, find a way to test whether the retrieved pose is right
    // set retrieved last position and retrieved current position
    // using retrieved drone position and current drone position to find a transform between
    // current drone frame and scene frame
    LOG(INFO)<<"AddRetrievedPose: mCONTROLLERSTATE: "<<mCONTROLLERSTATE11<<endl;
    if(mCONTROLLERSTATE11 == NO_SCENE_RETRIEVED_BEFORE)
    {
      LOG(INFO)<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE 1"<<endl;
        mCONTROLLERSTATE11 = SCENE_RETRIEVING_WORKING_NORMAL;

        LOG(INFO)<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE11"<<endl;

        mRetrievedPoseQueue11.push_back(retrieved_pose);

      LOG(INFO)<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE2"<<endl;
        mIdxMavScenes.push_back(make_tuple(mSceneRetrieveIndex, retrieved_pose, mMavrosPose11));
      LOG(INFO)<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE23"<<endl;
        mTest.push_back(make_tuple(mSceneRetrieveIndex, retrieved_pose, mMavrosPose11, mTargetPose));
      LOG(INFO)<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE24"<<endl;
        testfile<<mSceneRetrieveIndex<<","
                <<retrieved_pose.at<double>(0,3)<<","
                <<retrieved_pose.at<double>(1,3)<<","
                <<retrieved_pose.at<double>(2,3)<<","
                <<mMavrosPose11.pose.position.x<<","
                <<mMavrosPose11.pose.position.y<<","
                <<mMavrosPose11.pose.position.z<<","
                <<mTargetPose.pose.position.x<<","
                <<mTargetPose.pose.position.y<<","
                <<mTargetPose.pose.position.z<<endl;

        return;
    }

    // normal working state
    // prev and current retrieved pose were set
    // prev and current mavros pose at the time when scene was retrieved were set
    else if(mCONTROLLERSTATE11 == SCENE_RETRIEVING_WORKING_NORMAL || mCONTROLLERSTATE11 == MAVROS_STATE_ERROR)
    {
        LOG(INFO)<<"AddRetrievedPose: SCENE_RETRIEVING_WORKING_NORMAL"<<endl;

        LOG(INFO)<<"mRetrievedPoseQueue: "<<mRetrievedPoseQueue11.size()<<endl;

        cout<<"Controller::AddRetrievedPose, SCENE_RETRIEVING_WORKING_NORMAL"<<endl;

        if(true)
        //if(isSceneRecoveredMovementValid())
        {
            std::cout<<"Current retrieved pose is valid!"<<std::endl;

            mRetrievedPoseQueue11.push_back(retrieved_pose);

            mIdxMavScenes.push_back(make_tuple(mSceneRetrieveIndex, retrieved_pose, mMavrosPose11));

            mTest.push_back(make_tuple(mSceneRetrieveIndex, retrieved_pose, mMavrosPose11, mTargetPose));

            testfile<<mSceneRetrieveIndex<<","
                    <<retrieved_pose.at<double>(0,3)<<","
                    <<retrieved_pose.at<double>(1,3)<<","
                    <<retrieved_pose.at<double>(2,3)<<","
                    <<mMavrosPose11.pose.position.x<<","
                    <<mMavrosPose11.pose.position.y<<","
                    <<mMavrosPose11.pose.position.z<<","
                    <<mTargetPose.pose.position.x<<","
                    <<mTargetPose.pose.position.y<<","
                    <<mTargetPose.pose.position.z<<endl;

            if (mRetrievedPoseQueue11.size() > 3)
            {
                mIdxMavScenes.pop_front();
            }

            // if current retrieved pose from the scene is valid, current pose of the drone (frome SLAM)
            // could be updated, we can find a transform from current drone pose to retrieved scene pose
            // rather than updating current drone pose with this transform, we update the current target of
            // the drone with this transform.

            // No target received yet
            if(mTARGET == NO_TARGET)
                return;
            else
                UpdateTarget();
        }
        else
            std::cout<<"Current retrieved pose is not valid"<<std::endl;
    }
    else{
        cout<<"Current state: "<<mCONTROLLERSTATE11<<endl;
    }



    mSceneRetrieveIndex ++;
}


bool Controller::isSceneRecoveredMovementValid()
{
    float delta_retrieved = abs(mSceneRetrievedPosition.at<double>(0, 3) - mSceneRetrievedLastPosition.at<double>(0, 3)) +
                            abs(mSceneRetrievedPosition.at<double>(1, 3) - mSceneRetrievedLastPosition.at<double>(1, 3)) +
                            abs(mSceneRetrievedPosition.at<double>(2, 3) - mSceneRetrievedLastPosition.at<double>(2, 3));

    float delta_mavros = abs(mMavPoseCurRetrieved.pose.position.x - mMavPoseLastRetrieved.pose.position.x) +
                         abs(mMavPoseCurRetrieved.pose.position.y - mMavPoseLastRetrieved.pose.position.y) +
                         abs(mMavPoseCurRetrieved.pose.position.z - mMavPoseLastRetrieved.pose.position.z);

    float factor = abs(delta_mavros / delta_retrieved);

    if (factor < 3.0)
        return true;
    else
        return false;
}

void Controller::UpdateTarget()
{
    mTARGET == NEW_TARGET;

    if (mRetrievedPoseQueue.empty())
        return;

    float angles = 0;
    Eigen::Vector3d axises(0, 0,0 );
    Eigen::Vector3d position(0, 0, 0);
    for(auto& elem : mIdxMavScenes)
    {
        cv::Mat scene_pose_mat = std::get<1>(elem);
        geometry_msgs::PoseStamped mav_pose = std::get<2>(elem);
        cv::Mat mav_pose_mat = PoseStampedToMat(mav_pose);

        if(mav_pose_mat.empty() || scene_pose_mat.empty())
            return;

        cv::Mat RelativeTransform = findRelativeTransform(mav_pose_mat, scene_pose_mat);

        cv::Mat temp_rotation = RelativeTransform.colRange(0, 3).rowRange(0, 3);
        Eigen::Matrix3d rotation_eigen;
        cv::cv2eigen(temp_rotation, rotation_eigen);

        Eigen::AngleAxisd angle_axis(rotation_eigen);
        Eigen::Vector3d axis = angle_axis.axis();
        float angle = angle_axis.angle();

        axises[0] += axis[0];
        axises[1] += axis[1];
        axises[2] += axis[2];

        angles += angle;

        position[0] += RelativeTransform.at<double>(0, 3);
        position[1] += RelativeTransform.at<double>(1, 3);
        position[2] += RelativeTransform.at<double>(2, 3);
    }

    axises[0] = axises[0] / mIdxMavScenes.size();
    axises[1] = axises[1] / mIdxMavScenes.size();
    axises[2] = axises[2] / mIdxMavScenes.size();
    angles = angles / mIdxMavScenes.size();
    position[0] = position[0] / mIdxMavScenes.size();
    position[1] = position[1] / mIdxMavScenes.size();
    position[2] = position[2] / mIdxMavScenes.size();

    Eigen::AngleAxisd averaged_rotation_angleaxis(angles, axises);
    Eigen::Matrix3d averaged_rotation_eigen = averaged_rotation_angleaxis.toRotationMatrix();
    cv::Mat averaged_rotation_mat;
    cv::eigen2cv(averaged_rotation_eigen, averaged_rotation_mat);

    cv::Mat relative_transform = cv::Mat::zeros(4,4, CV_64F);

    relative_transform.at<double>(0, 0) = averaged_rotation_mat.at<double>(0, 0);
    relative_transform.at<double>(0, 1) = averaged_rotation_mat.at<double>(0, 1);
    relative_transform.at<double>(0, 2) = averaged_rotation_mat.at<double>(0, 2);
    relative_transform.at<double>(1, 0) = averaged_rotation_mat.at<double>(1, 0);
    relative_transform.at<double>(1, 1) = averaged_rotation_mat.at<double>(1, 1);
    relative_transform.at<double>(1, 2) = averaged_rotation_mat.at<double>(1, 2);
    relative_transform.at<double>(2, 0) = averaged_rotation_mat.at<double>(2, 0);
    relative_transform.at<double>(2, 1) = averaged_rotation_mat.at<double>(2, 1);
    relative_transform.at<double>(2, 2) = averaged_rotation_mat.at<double>(2, 2);
    relative_transform.at<double>(0, 3) = position[0];
    relative_transform.at<double>(1, 3) = position[1];
    relative_transform.at<double>(2, 3) = position[2];
    relative_transform.at<double>(3, 3) = 1.0;

    if (mSceneRetrieveIndex <= 3)
    {
        //initial transform is mavros to scene
        mInitialRelativeTransform = relative_transform;

        LOG(INFO)<<"mInitialRelativeTransform: "<<mInitialRelativeTransform<<endl;
    }
    else{
        mCurrentRelativeTransform = relative_transform;

        if(mCurrentRelativeTransform.empty() || mInitialRelativeTransform.empty())
            return;

        //current transform is from mavros to scene
        cv::Mat current2initial = findRelativeTransform(mCurrentRelativeTransform, mInitialRelativeTransform);

        cv::Mat targetMat = PoseStampedToMat(mTargetPose);
        cv::Mat initial2current = current2initial.inv();
        cv::Mat updatedTargetMat = current2initial * targetMat;

        geometry_msgs::PoseStamped result_target = MatToPoseStamped(updatedTargetMat, mTargetPose.header.frame_id);
        mTargetPose = result_target;
    }

}

//void Controller::UpdateTarget()
//{
//    if(mRetrievedPoseQueue.empty())
//        return;
//
//    // step 1, given a series of relative Transforms, find mean relative transform for these two frames
//    float angles = 0;
//    Eigen::Vector3d axises(0, 0,0 );
//    Eigen::Vector3d position(0, 0, 0);
//
//    for(auto& T : mRetrievedPoseQueue)
//    {
//        cv::Mat temp_rotation = T.colRange(0, 3).rowRange(0, 3);
//        Eigen::Matrix3d rotation_eigen;
//        cv::cv2eigen(temp_rotation, rotation_eigen);
//
//        Eigen::AngleAxisd angle_axis(rotation_eigen);
//        Eigen::Vector3d axis = angle_axis.axis();
//        float angle = angle_axis.angle();
//
//        axises[0] += axis[0];
//        axises[1] += axis[1];
//        axises[2] += axis[2];
//
//        angles += angle;
//
//        position[0] += T.at<double>(0, 3);
//        position[1] += T.at<double>(1, 3);
//        position[2] += T.at<double>(2, 3);
//    }
//
//    axises[0] = axises[0] / mRetrievedPoseQueue.size();
//    axises[1] = axises[1] / mRetrievedPoseQueue.size();
//    axises[2] = axises[2] / mRetrievedPoseQueue.size();
//    angles = angles / mRetrievedPoseQueue.size();
//    position[0] = position[0] / mRetrievedPoseQueue.size();
//    position[1] = position[1] / mRetrievedPoseQueue.size();
//    position[2] = position[2] / mRetrievedPoseQueue.size();
//
//    Eigen::AngleAxisd averaged_rotation_angleaxis(angles, axises);
//    Eigen::Matrix3d averaged_rotation_eigen = averaged_rotation_angleaxis.toRotationMatrix();
//    cv::Mat averaged_rotation_mat;
//    cv::eigen2cv(averaged_rotation_eigen, averaged_rotation_mat);
//
//    cv::Mat averaged_T = cv::Mat::zeros(4,4, CV_64F);
//
//    averaged_T.at<double>(0, 0) = averaged_rotation_mat.at<double>(0, 0);
//    averaged_T.at<double>(0, 1) = averaged_rotation_mat.at<double>(0, 1);
//    averaged_T.at<double>(0, 2) = averaged_rotation_mat.at<double>(0, 2);
//    averaged_T.at<double>(1, 0) = averaged_rotation_mat.at<double>(1, 0);
//    averaged_T.at<double>(1, 1) = averaged_rotation_mat.at<double>(1, 1);
//    averaged_T.at<double>(1, 2) = averaged_rotation_mat.at<double>(1, 2);
//    averaged_T.at<double>(2, 0) = averaged_rotation_mat.at<double>(2, 0);
//    averaged_T.at<double>(2, 1) = averaged_rotation_mat.at<double>(2, 1);
//    averaged_T.at<double>(2, 2) = averaged_rotation_mat.at<double>(2, 2);
//    averaged_T.at<double>(0, 3) = position[0];
//    averaged_T.at<double>(1, 3) = position[1];
//    averaged_T.at<double>(2, 3) = position[2];
//    averaged_T.at<double>(3, 3) = 1.0;
//
//    // step 2, find relative transform between current mavros pose and retrieved pose frame
//    // the result if Transform from mavros pose to scene retrieved pose frame
//    cv::Mat MavrosPoseMat = PoseStampedToMat(mMavPoseCurRetrieved);
//    cv::Mat RelativeTransform = findRelativeTransform(MavrosPoseMat, averaged_T);
//
//    if (mSceneRetrieveIndex <= 3) {
//        mInitialRelativeTransform = RelativeTransform;
//        LOG(INFO)<<"mInitialRelativeTransform: "<<mInitialRelativeTransform<<endl;
//    }
//    else{
//        mCurrentRelativeTransform = RelativeTransform;
//
//        // there is an initial relative Transform between mavros frame and scene frame,
//        // and a current relative Transform between these two frames,
//        // we find the transform between current relative transform and initial relative transform
//        // and convert current target to initial frame.
//        LOG(INFO)<<"mInitialRelativeTransform: "<<mInitialRelativeTransform<<", mCurrentRelativeTransform: "<<mCurrentRelativeTransform<<endl;
//
//        cv::Mat initial2current = findRelativeTransform(mInitialRelativeTransform, mCurrentRelativeTransform);
//
//        // step 3, convert target from SLAM frame to Scene Retreving frame (the true frame)
//        cv::Mat targetMat = PoseStampedToMat(mTargetPose);
//        cv::Mat updatedTargetMat = initial2current.inv() * targetMat;
//
//        geometry_msgs::PoseStamped result_target = MatToPoseStamped(updatedTargetMat, mTargetPose.header.frame_id);
//        mTargetPose = result_target;
//    }
//
//}

void Controller::TargetSetSubCallback(const geometry_msgs::PoseStamped& target)
{
//    geometry_msgs::PoseStamped test;
//    test.pose.position.x = 0;
//    test.pose.position.y = -10;
//    test.pose.position.z = 3;
//    mTargetPose = test;

    mTargetPose = target;
    mTargetPose.pose.position.z = 3;

    LOG(INFO)<<"Received new target: "<<mTargetPose.pose.position.x<<", "<<
                                        mTargetPose.pose.position.y<<", "<<
                                        mTargetPose.pose.position.z<<endl;
    //UpdateTarget();
    //GoToTarget(mTargetPose);

    // enable new task after each targetset callback
    mTARGET = NEW_TARGET;
}

void Controller::MavrosPoseCallback(const geometry_msgs::PoseStamped& pose)
{
    LOG(INFO)<<"mavros pose callback!"<<endl;
    mMavrosInitialized = true;

    mLastMavrosPose = mMavrosPose;
    mMavrosPose = pose;

//    if (!isMavrosPoseValid())
//        mCONTROLLERSTATE = MAVROS_STATE_ERROR;
}

bool Controller::isMavrosPoseValid()
{

    float delta_t = (mMavrosPose.header.stamp - mLastMavrosPose.header.stamp).toSec();

    if(delta_t == 0)
    {
        return true;
    }

    float distance_squared =
            pow(abs(mMavrosPose.pose.position.x - mLastMavrosPose.pose.position.x), 2) +
            pow(abs(mMavrosPose.pose.position.y - mLastMavrosPose.pose.position.y), 2) +
            pow(abs(mMavrosPose.pose.position.z - mLastMavrosPose.pose.position.z), 2);

    float speed = sqrt(distance_squared) / delta_t;
    
    // the horizontal flight speed of drone is less than 12 by default
    if (speed > 10)
    {
        mCONTROLLERSTATE = MAVROS_STATE_ERROR;
        return false;
    }

    float cur_yaw, prev_yaw;

    Eigen::Quaternionf cur_rotation_matrix(mMavrosPose.pose.orientation.w,
                                           mMavrosPose.pose.orientation.x,
                                           mMavrosPose.pose.orientation.y,
                                           mMavrosPose.pose.orientation.z);
    Eigen::Vector3f euler_cur = cur_rotation_matrix.toRotationMatrix().eulerAngles(0, 1, 2);
    cur_yaw = euler_cur[2];

    Eigen::Quaternionf prev_rotation_matrix(mLastMavrosPose.pose.orientation.w,
                                            mLastMavrosPose.pose.orientation.x,
                                            mLastMavrosPose.pose.orientation.y,
                                            mLastMavrosPose.pose.orientation.z);
    Eigen::Vector3f euler_prev = prev_rotation_matrix.toRotationMatrix().eulerAngles(0, 1, 2);
    prev_yaw = euler_prev[2];

    float yaw_rate = (cur_yaw - prev_yaw) / delta_t;
    
    // the yaw change rate is less than pi by default
    if (yaw_rate > 2*3.1415926)
    {
        mCONTROLLERSTATE = MAVROS_STATE_ERROR;
        return false;
    }

    return true;
}

void Controller::Hover()
{

}

//cv::Mat Controller::findRelativeTransform(cv::Mat& Twb1, cv::Mat& Twb2)

