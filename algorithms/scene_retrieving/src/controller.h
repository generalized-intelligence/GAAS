#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <queue>
#include <cmath>
#include <thread>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

class Controller{

public:

    Controller(ros::NodeHandle& nh);

    void Run();

    void SetTarget(geometry_msgs::PoseStamped& target);

    bool GoToTarget(const geometry_msgs::PoseStamped& target, bool useBodyFrame=false);

    void AddRetrievedPose(Eigen::Vector4f& retrieved_pose);

    bool isSceneRecoveredMovementValid();

    bool isMavrosPoseValid();

    void MavrosPoseCallback(const geometry_msgs::PoseStamped& pose);

    void TargetSetSubCallback(const geometry_msgs::PoseStamped& target);

    void UpdateCurrentPose();

    void UpdateTarget();

    void Hover();

    enum mState{
        NO_SCENE_RETRIEVED_BEFORE,
        SCENE_RETRIEVING_WORKING_NORMAL,
        SCENE_RETRIEVING_INTERMITTENT,
        MAVROS_STATE_ERROR,
    };

    enum mTarget{
        NO_TARGET,
        NEW_TARGET,
        TARGET_REACHED,
    };

private:

    int mLoopIndex = 0;

    geometry_msgs::PoseStamped mCurMavrosPose;
    geometry_msgs::PoseStamped mLastMavrosPose;

    Eigen::Vector4f mSceneRetrievedPosition;
    Eigen::Vector4f mSceneRetrievedLastPosition;
    geometry_msgs::PoseStamped mMavPoseLastRetrieved;
    geometry_msgs::PoseStamped mMavPoseCurRetrieved;

    geometry_msgs::PoseStamped mMavrosPose;
    geometry_msgs::PoseStamped mTargetPose;

    Eigen::Vector4f mUpdatedCurrentPose;
    Eigen::Vector4f mCurrentDistanceToTarget;

    // Transformation from drone to scene
    cv::Mat mTscene_drone;

    ros::Subscriber mMavrosSub;
    ros::Subscriber mTargetSetSub;
    ros::Publisher mPositionControlPub;
    ros::Publisher mYawControlPub;


    //    ros::Subscriber mSub;
    //    ros::Subscriber mSubCamera_3;

    queue<Eigen::Vector4f> mRetrievedPoseQueue;

    mState mSTATE;
    mTarget mTARGET;

    ros::NodeHandle mNH;
};



#endif