#include "controller.h"


Controller::Controller(ros::NodeHandle& nh)
{
    mNH = nh;
    mMavrosSub = mNH.subscribe("/mavros/local_position/pose", 100, &Controller::MavrosPoseCallback, this);
    mTargetSetSub = mNH.subscribe("/move_base_simple/goal", 100, &Controller::TargetSetSubCallback, this);
    mPositionControlPub = mNH.advertise<geometry_msgs::PoseStamped>("gi/set_pose/position", 100);
    mYawControlPub = mNH.advertise<std_msgs::Float32>("gi/set_pose/orientation", 100);

    mSceneRetrievedPosition = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
    mTargetPose.pose.position.x = 0;
    mTargetPose.pose.position.y = 0;
    mTargetPose.pose.position.z = 0;

    mSTATE = mState::NO_SCENE_RETRIEVED_BEFORE;
    mTARGET = mTarget::NO_TARGET;

    std::thread t(&Controller::Run, this);
    t.detach();
}

void Controller::Run()
{
    ros::Rate rate(10.);
    while (!ros::isShuttingDown())
    {

        //constantly fetching information from MAVROS
        if(mSTATE == MAVROS_STATE_ERROR)
        {
            Hover();
        }

        if(mTARGET == NO_TARGET)
        {
            continue;
        }
        else if (mTARGET == NEW_TARGET)
        {
            if(!mRetrievedPoseQueue.empty())
            {
                UpdateTarget();
            }
            GoToTarget(mTargetPose);
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

bool Controller::GoToTarget(const geometry_msgs::PoseStamped& target, bool useBodyFrame)
{

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

    cout<<"Going to target: "<<pose.pose.position.x<<", "
                             <<pose.pose.position.y<<", "
                             <<pose.pose.position.z<<endl;

    mPositionControlPub.publish(pose);
}

void Controller::AddRetrievedPose(cv::Mat& retrieved_pose)
{
    mSceneRetrievedLastPosition = mSceneRetrievedPosition;
    mSceneRetrievedPosition = retrieved_pose;

    mMavPoseLastRetrieved = mMavPoseCurRetrieved;
    mMavPoseCurRetrieved = mCurMavrosPose;

    if (abs(mCurMavrosPose.pose.position.x)+abs(mCurMavrosPose.pose.position.y)+abs(mCurMavrosPose.pose.position.z)==0)
    {
        LOG(WARNING)<<"Current Mavros Pose not valid, adding retrieved pose failed!"<<endl;
        return;
    }

    // the first retrieved pose
    // NOTE assuming the first retrieved pose is "right" TODO, find a way to test whether the retrieved pose is right
    // set retrieved last position and retrieved current position
    // set drone position at present
    // using retrieved drone position and current drone position to find a transform between
    // current drone frame and scene frame
    if(mSTATE == NO_SCENE_RETRIEVED_BEFORE)
    {
        mSTATE = SCENE_RETRIEVING_WORKING_NORMAL;

        cout<<"AddRetrievedPose: NO_SCENE_RETRIEVED_BEFORE"<<endl;

//        // RelativeTransform: from body to retrieved frame
//        cv::Mat MavrosPoseMat = PoseStampedToMat(mMavPoseCurRetrieved);
//        cv::Mat RelativeTransform = findRelativeTransform(MavrosPoseMat, mSceneRetrievedPosition);
//
//        if(mSceneRetrieveIndex <= 5)
//            mRelativeTransforms.emplace_back(RelativeTransform);



        mRetrievedPoseVec.emplace_back(retrieved_pose);

        return;
    }
    // normal working state
    // prev and current retrieved pose were set
    // prev and current mavros pose at the time when scene was retrieved were set
    else if(mSTATE == SCENE_RETRIEVING_WORKING_NORMAL)
    {

        cout<<"AddRetrievedPose: SCENE_RETRIEVING_WORKING_NORMAL"<<endl;

        if(isSceneRecoveredMovementValid())
        {
            std::cout<<"Current retrieved pose is valid!"<<std::endl;

//            // RelativeTransform: from body to retrieved frame
//            cv::Mat MavrosPoseMat = PoseStampedToMat(mMavPoseCurRetrieved);
//            cv::Mat RelativeTransform = findRelativeTransform(MavrosPoseMat, mSceneRetrievedPosition);
//
//            if(mSceneRetrieveIndex <= 5)
//                mRelativeTransforms.emplace_back(RelativeTransform);


            mRetrievedPoseVec.emplace_back(retrieved_pose);

            // if current retrieved pose from the scene is valid, current pose of the drone (frome SLAM)
            // could be updated, we can find a transform from current drone pose to retrieved scene pose
            // rather than updating current drone pose with this transform, we update the current target of
            // the drone with this transform.
            UpdateTarget();
        }
        else
            std::cout<<"Current retrieved pose is not valid"<<std::endl;
    } else{

        cout<<"Current state: "<<mSTATE<<endl;
    }

    mSceneRetrieveIndex ++;

    mRetrievedPoseQueue.push(retrieved_pose);
}


bool Controller::isSceneRecoveredMovementValid()
{
    float delta_retrieved = abs(mSceneRetrievedPosition.at<double>(0, 3) - mSceneRetrievedLastPosition.at<double>(0, 3)) +
                            abs(mSceneRetrievedPosition.at<double>(1, 3) - mSceneRetrievedLastPosition.at<double>(1, 3)) +
                            abs(mSceneRetrievedPosition.at<double>(2, 3) - mSceneRetrievedLastPosition.at<double>(2, 3));

    float delta_mavros = abs(mMavPoseCurRetrieved.pose.position.x - mMavPoseLastRetrieved.pose.position.x) +
                         abs(mMavPoseCurRetrieved.pose.position.y - mMavPoseLastRetrieved.pose.position.y) +
                         abs(mMavPoseCurRetrieved.pose.position.z - mMavPoseLastRetrieved.pose.position.z);

//    if (abs(1.0 - delta_retrieved / delta_mavros) < 0.05)
//        return true;
//    else
//        return false;

    float factor = abs(delta_mavros / delta_retrieved);
    cout<<"isSceneRecoveredMovementValid: "<<delta_mavros<<", "<<delta_retrieved<<", "<<factor<<endl;

    if (factor < 3.0)
        return true;
    else
        return false;

}

//void Controller::UpdateTarget()
//{
//    mSceneRetrievedLastPosition = mSceneRetrievedPosition;
//    //mSceneRetrievedPosition = retrieved_pose;
//
//    mMavPoseLastRetrieved = mMavPoseCurRetrieved;
//    mMavPoseCurRetrieved = mCurMavrosPose;
//
//    // delta x, y and z from retrieved pose to drone pose
//    float delta_x = mSceneRetrievedPosition.at<double>(0, 3) - mMavPoseCurRetrieved.pose.position.x;
//    float delta_y = mSceneRetrievedPosition.at<double>(2, 3) - mMavPoseCurRetrieved.pose.position.y;
//    float delta_z = mSceneRetrievedPosition.at<double>(3, 3) - mMavPoseCurRetrieved.pose.position.z;
//
//    mTargetPose.pose.position.x = mTargetPose.pose.position.x - delta_x;
//    mTargetPose.pose.position.y = mTargetPose.pose.position.y - delta_y;
//    mTargetPose.pose.position.z = mTargetPose.pose.position.z - delta_z;
//}

void Controller::UpdateTarget()
{
    if(mRelativeTransforms.empty())
        return;

    // step 1, given a series of relative Transforms, find mean relative transform for these two frames
    float angles;
    Eigen::Vector3d axises(0, 0,0 );
    Eigen::Vector3d position(0, 0, 0);

    LOG(INFO)<<"UpdateTarget"<<endl;
    LOG(INFO)<<"mRetrievedPoseVec size: "<<mRetrievedPoseVec.size()<<endl;

    if (mRetrievedPoseVec.empty())
        return; // This should not happen

    for(auto& T : mRetrievedPoseVec)
    {
        cv::Mat temp_rotation = T.colRange(0, 3).rowRange(0, 3);
        Eigen::Matrix3d rotation_eigen;
        cv::cv2eigen(temp_rotation, rotation_eigen);

        Eigen::AngleAxisd angle_axis(rotation_eigen);
        Eigen::Vector3d axis = angle_axis.axis();
        float angle = angle_axis.angle();

        LOG(INFO)<<"T: "<<T<<endl;
        LOG(INFO)<<"rotation_eigen: "<<rotation_eigen<<endl;
        LOG(INFO)<<"angle and axis: "<<angle<<", "<<axis<<endl;

        axises[0] += axis[0];
        axises[1] += axis[1];
        axises[2] += axis[2];

        angles += angle;

        position[0] += T.at<double>(0, 3);
        position[1] += T.at<double>(1, 3);
        position[2] += T.at<double>(2, 3);
    }

    axises[0] = axises[0] / mRetrievedPoseVec.size();
    axises[1] = axises[1] / mRetrievedPoseVec.size();
    axises[2] = axises[2] / mRetrievedPoseVec.size();
    angles = angles / mRetrievedPoseVec.size();

    position[0] = position[0] / mRetrievedPoseVec.size();
    position[1] = position[1] / mRetrievedPoseVec.size();
    position[2] = position[2] / mRetrievedPoseVec.size();

    cout<<"Averaged axise value and angle value are: "<<axises<<", "<<angles<<endl;
    Eigen::AngleAxisd averaged_rotation_angleaxis(angles, axises);
    Eigen::Matrix3d averaged_rotation_eigen = averaged_rotation_angleaxis.toRotationMatrix();
    cv::Mat averaged_rotation_mat;
    cv::eigen2cv(averaged_rotation_eigen, averaged_rotation_mat);

    cv::Mat averaged_T = cv::Mat::zeros(4,4, CV_64FC1);

    averaged_T.at<double>(0, 0) = averaged_rotation_mat.at<double>(0, 0);
    averaged_T.at<double>(0, 1) = averaged_rotation_mat.at<double>(0, 1);
    averaged_T.at<double>(0, 2) = averaged_rotation_mat.at<double>(0, 2);
    averaged_T.at<double>(1, 0) = averaged_rotation_mat.at<double>(1, 0);
    averaged_T.at<double>(1, 1) = averaged_rotation_mat.at<double>(1, 1);
    averaged_T.at<double>(1, 2) = averaged_rotation_mat.at<double>(1, 2);
    averaged_T.at<double>(2, 0) = averaged_rotation_mat.at<double>(2, 0);
    averaged_T.at<double>(2, 1) = averaged_rotation_mat.at<double>(2, 1);
    averaged_T.at<double>(2, 2) = averaged_rotation_mat.at<double>(2, 2);
    averaged_T.at<double>(0, 3) = position[0];
    averaged_T.at<double>(1, 3) = position[1];
    averaged_T.at<double>(2, 3) = position[2];
    averaged_T.at<double>(3, 3) = 1.0;

    // step 2, find relative transform between current mavros pose and retrieved pose frame
    // the result if Transform from mavros pose to scene retrieved pose frame
    cv::Mat MavrosPoseMat = PoseStampedToMat(mMavPoseCurRetrieved);
    cv::Mat RelativeTransform = findRelativeTransform(MavrosPoseMat, averaged_T);

    if (mSceneRetrieveIndex <= 3)
        mInitialRelativeTransform = RelativeTransform;
    else
        mCurrentRelativeTransform = RelativeTransform;

    // there is an initial relative Transform between mavros frame and scene frame,
    // and a current relative Transform between these two frames,
    // we find the transform between current relative transform and initial relative transform
    // and convert current target to initial frame.
    cv::Mat initial2current = findRelativeTransform(mInitialRelativeTransform, mCurrentRelativeTransform);

    // step 3, convert target from SLAM frame to Scene Retreving frame (the true frame)
    cv::Mat targetMat = PoseStampedToMat(mTargetPose);
    cv::Mat updatedTargetMat = initial2current.inv() * targetMat;

    geometry_msgs::PoseStamped result_target = MatToPoseStamped(updatedTargetMat, mTargetPose.header.frame_id);
    mTargetPose = result_target;
    
}

void Controller::TargetSetSubCallback(const geometry_msgs::PoseStamped& target)
{
    mTargetPose = target;
    cout<<"Received new target: "<<mTargetPose.pose.position.x<<", "<<
                                   mTargetPose.pose.position.y<<", "<<
                                   mTargetPose.pose.position.z<<endl;
    //UpdateTarget();
    GoToTarget(mTargetPose);
    mTARGET = NEW_TARGET;
}

void Controller::MavrosPoseCallback(const geometry_msgs::PoseStamped& pose)
{
    mLastMavrosPose = mCurMavrosPose;
    mCurMavrosPose = pose;

    if (!isMavrosPoseValid())
        mSTATE = MAVROS_STATE_ERROR;
}

bool Controller::isMavrosPoseValid()
{

    float delta_t = (mCurMavrosPose.header.stamp - mLastMavrosPose.header.stamp).toSec();

    if(delta_t == 0)
    {
        return true;
    }

    float distance_squared =
            pow(abs(mCurMavrosPose.pose.position.x - mLastMavrosPose.pose.position.x), 2) +
            pow(abs(mCurMavrosPose.pose.position.y - mLastMavrosPose.pose.position.y), 2) +
            pow(abs(mCurMavrosPose.pose.position.z - mLastMavrosPose.pose.position.z), 2);

    float speed = sqrt(distance_squared) / delta_t;
    
    // the horizontal flight speed of drone is less than 12 by default
    if (speed > 10)
    {
        mSTATE = MAVROS_STATE_ERROR;
        return false;
    }

    float cur_yaw, prev_yaw;

    Eigen::Quaternionf cur_rotation_matrix(mCurMavrosPose.pose.orientation.w,
                                           mCurMavrosPose.pose.orientation.x,
                                           mCurMavrosPose.pose.orientation.y,
                                           mCurMavrosPose.pose.orientation.z);
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
        mSTATE = MAVROS_STATE_ERROR;
        return false;
    }

    return true;
}

void Controller::Hover()
{

}

//cv::Mat Controller::findRelativeTransform(cv::Mat& Twb1, cv::Mat& Twb2)

