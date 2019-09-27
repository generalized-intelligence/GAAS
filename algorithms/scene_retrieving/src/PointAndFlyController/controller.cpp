#include "controller.h"


Controller::Controller(ros::NodeHandle& nh)
{

    mNH = nh;
    mMavrosSub = mNH.subscribe("/mavros/local_position/pose", 100, &Controller::MavrosPoseCallback, this);
    mTargetSetSub = mNH.subscribe("/move_base_simple/goal", 100, &Controller::TargetSetSubCallback, this);
    mPositionControlPub = mNH.advertise<geometry_msgs::PoseStamped>("gi/set_pose/position", 100);
    mYawControlPub = mNH.advertise<std_msgs::Float32>("gi/set_pose/orientation", 100);

    mSceneRetrievedPosition.setZero();
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
    //while (!ros::isShuttingDown())
    while (ros::isShuttingDown())
    {

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

            if(mRetrievedPoseQueue.empty())
            {
                GoToTarget(mTargetPose);
            }
            else
            {
                Eigen::Vector4f latest_retrieved_pose = mRetrievedPoseQueue.back();
                mRetrievedPoseQueue.pop();
                
                // if retrieved pose from scene is valid, we can use it to update target
                if(isSceneRecoveredMovementValid())
                {
                    UpdateTarget();
                    GoToTarget(mTargetPose);
                }
                // if retrieved pose from scene is not valid, ignore and continue current target
                else
                {
                    GoToTarget(mTargetPose);
                }
            }
        }
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

void Controller::AddRetrievedPose(Eigen::Vector4f& retrieved_pose)
{
    mSceneRetrievedLastPosition = mSceneRetrievedPosition;
    mSceneRetrievedPosition = retrieved_pose;

    // neglect the first retrieved pose
    // set retrieved last position and retrieved current position
    // set mavros pose when previously add retrieved pose and when current retrieved pose
    if(mSTATE == NO_SCENE_RETRIEVED_BEFORE)
    {
        mSceneRetrievedLastPosition = mSceneRetrievedPosition;
        mSceneRetrievedPosition = retrieved_pose;

        mSTATE = SCENE_RETRIEVING_WORKING_NORMAL;

        mMavPoseLastRetrieved = mMavPoseCurRetrieved;
        mMavPoseCurRetrieved = mCurMavrosPose;

        return;
    }
    // normal working state
    // prev and current retrieved pose were set
    // prev and current mavros pose at the time when scene was retrieved were set
    else if(mSTATE == SCENE_RETRIEVING_WORKING_NORMAL)
    {
        mSceneRetrievedLastPosition = mSceneRetrievedPosition;
        mSceneRetrievedPosition = retrieved_pose;

        mMavPoseLastRetrieved = mMavPoseCurRetrieved;
        mMavPoseCurRetrieved = mCurMavrosPose;

        if(isSceneRecoveredMovementValid())
        {
            std::cout<<"Current retrieved pose is valid!"<<std::endl;

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

    mRetrievedPoseQueue.push(retrieved_pose);
}


bool Controller::isSceneRecoveredMovementValid()
{
    float delta_retrieved = abs(mSceneRetrievedPosition[0] - mSceneRetrievedLastPosition[0]) +
                            abs(mSceneRetrievedPosition[1] - mSceneRetrievedLastPosition[1]) +
                            abs(mSceneRetrievedPosition[2] - mSceneRetrievedLastPosition[2]);

    float delta_mavros = abs(mMavPoseCurRetrieved.pose.position.x - mMavPoseLastRetrieved.pose.position.x) +
                         abs(mMavPoseCurRetrieved.pose.position.y - mMavPoseLastRetrieved.pose.position.y) +
                         abs(mMavPoseCurRetrieved.pose.position.z - mMavPoseLastRetrieved.pose.position.z);

    if (abs(1.0 - delta_retrieved / delta_mavros) < 0.05)
        return true;
    else
        return false;
}

void Controller::UpdateTarget()
{
    mSceneRetrievedLastPosition = mSceneRetrievedPosition;
    //mSceneRetrievedPosition = retrieved_pose;

    mMavPoseLastRetrieved = mMavPoseCurRetrieved;
    mMavPoseCurRetrieved = mCurMavrosPose;

    // delta x, y and z from retrieved pose to drone pose
    float delta_x = mSceneRetrievedPosition[0] - mMavPoseCurRetrieved.pose.position.x;
    float delta_y = mSceneRetrievedPosition[1] - mMavPoseCurRetrieved.pose.position.y;
    float delta_z = mSceneRetrievedPosition[2] - mMavPoseCurRetrieved.pose.position.z;

    mTargetPose.pose.position.x = mTargetPose.pose.position.x - delta_x;
    mTargetPose.pose.position.y = mTargetPose.pose.position.y - delta_y;
    mTargetPose.pose.position.z = mTargetPose.pose.position.z - delta_z;
}

void Controller::MavrosPoseCallback(const geometry_msgs::PoseStamped& pose)
{
    mLastMavrosPose = mCurMavrosPose;
    mCurMavrosPose = pose;

    if (!isMavrosPoseValid())
        mSTATE = MAVROS_STATE_ERROR;
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

bool Controller::isMavrosPoseValid()
{

    float  delta_t = (mCurMavrosPose.header.stamp - mLastMavrosPose.header.stamp).toSec();

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

    float cur_roll, cur_pitch, cur_yaw;
    //tf::Matrix3x3(mCurMavrosPose.pose.orientation).getRPY(cur_roll, cur_pitch, cur_yaw);

    float prev_roll, prev_pitch, prev_yaw;
    //tf::Matrix3x3(mLastMavrosPose.pose.orientation).getRPY(prev_roll, prev_pitch, prev_yaw);

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

    cout<<"Current Drone speed: "<<speed<<", yaw rate: "<<yaw_rate<<endl;
    
    // the yaw change rate is less than pi by default
    if (yaw_rate > 3.1415926)
    {
        mSTATE = MAVROS_STATE_ERROR;
        return false;
    }

    return true;
}

void Controller::Hover()
{

}
