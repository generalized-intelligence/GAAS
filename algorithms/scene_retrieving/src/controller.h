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

#include <glog/logging.h>

#include <opencv2/core/eigen.hpp>

using namespace std;

class Controller{

public:

    Controller(ros::NodeHandle& nh);

    void Run();

    void SetTarget(geometry_msgs::PoseStamped& target);

    bool GoToTarget(const geometry_msgs::PoseStamped& target, bool useBodyFrame=false);

    void AddRetrievedPose(cv::Mat& retrieved_pose);

    bool isSceneRecoveredMovementValid();

    bool isMavrosPoseValid();

    void MavrosPoseCallback(const geometry_msgs::PoseStamped& pose);

    void TargetSetSubCallback(const geometry_msgs::PoseStamped& target);

    void UpdateCurrentPose();

    void UpdateTarget();

    void Hover();

    inline cv::Mat PoseStampedToMat(const geometry_msgs::PoseStamped& pose)
    {
        Eigen::Quaterniond pose_q(pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z);

        Eigen::Matrix3d pose_R = pose_q.toRotationMatrix();

        cv::Mat mat_R;
        cv::eigen2cv(pose_R, mat_R);

        cv::Mat T = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);

        T.at<double>(0, 0) = mat_R.at<double>(0, 0);
        T.at<double>(0, 1) = mat_R.at<double>(0, 1);
        T.at<double>(0, 2) = mat_R.at<double>(0, 2);
        T.at<double>(1, 0) = mat_R.at<double>(1, 0);
        T.at<double>(1, 1) = mat_R.at<double>(1, 1);
        T.at<double>(1, 2) = mat_R.at<double>(1, 2);
        T.at<double>(2, 0) = mat_R.at<double>(2, 0);
        T.at<double>(2, 1) = mat_R.at<double>(2, 1);
        T.at<double>(2, 2) = mat_R.at<double>(2, 2);

        T.at<double>(0, 3) = pose.pose.position.x;
        T.at<double>(1, 3) = pose.pose.position.y;
        T.at<double>(2, 3) = pose.pose.position.z;
        T.at<double>(3, 3) = 1;

        cout<<"PoseStampedToMat: \n"<<T<<endl;

        return T;
    }

    inline geometry_msgs::PoseStamped MatToPoseStamped(cv::Mat& pose_mat, string& frame_id)
    {
        cv::Mat rotation_mat = pose_mat.colRange(0, 3).rowRange(0, 3);
        Eigen::Matrix3d rotation_eigen;
        cv::cv2eigen(rotation_mat, rotation_eigen);
        Eigen::Quaterniond quat(rotation_eigen);

        geometry_msgs::PoseStamped result_pose;
        result_pose.header.stamp = ros::Time::now();
        result_pose.header.frame_id = frame_id;
        result_pose.pose.orientation.w = quat.w();
        result_pose.pose.orientation.x = quat.x();
        result_pose.pose.orientation.y = quat.y();
        result_pose.pose.orientation.z = quat.z();
        result_pose.pose.position.x = pose_mat.at<double>(0, 3);
        result_pose.pose.position.y = pose_mat.at<double>(1, 3);
        result_pose.pose.position.z = pose_mat.at<double>(2, 3);

        return result_pose;
    }

    inline cv::Mat findRelativeTransform(cv::Mat& Twb1, cv::Mat& Twb2)
    {
//        //NOTE they should be body to world, or Twb
//        cv::Mat Tb1b2;
//        cv::Mat Tb1w = Twb1.inv();
//        Tb1b2 = Tb1w * Twb2;
//
//        cout<<"Twb1: \n"<<Twb1<<endl;
//        cout<<"Twb2: \n"<<Twb2<<endl;
//        cout<<"Tb1b2: \n"<<Tb1b2<<endl;
//
//        return Tb1b2;

        //NOTE they should be body to world, or Twb
        cv::Mat Tb2b1;
        cv::Mat Tb2w = Twb2.inv();
        Tb2b1 = Tb2w * Twb1;

        cout<<"Twb1: \n"<<Twb1<<endl;
        cout<<"Twb2: \n"<<Twb2<<endl;
        cout<<"Tb1b2: \n"<<Tb2b1<<endl;

        return Tb2b1;

    }


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

    cv::Mat mSceneRetrievedPosition;
    cv::Mat mSceneRetrievedLastPosition;

    geometry_msgs::PoseStamped mMavPoseLastRetrieved;
    geometry_msgs::PoseStamped mMavPoseCurRetrieved;

    geometry_msgs::PoseStamped mMavrosPose;
    geometry_msgs::PoseStamped mTargetPose;

    cv::Mat mUpdatedCurrentPose;
    cv::Mat mCurrentDistanceToTarget;

    cv::Mat mInitialRelativeTransform;
    cv::Mat mCurrentRelativeTransform;

    // Transformation from drone to scene
    cv::Mat mTscene_drone;

    ros::Subscriber mMavrosSub;
    ros::Subscriber mTargetSetSub;
    ros::Publisher mPositionControlPub;
    ros::Publisher mYawControlPub;

    size_t mSceneRetrieveIndex = 0;

    //    ros::Subscriber mSub;
    //    ros::Subscriber mSubCamera_3;

    queue<cv::Mat> mRetrievedPoseQueue;
    vector<cv::Mat> mRetrievedPoseVec;

    vector<cv::Mat> mRelativeTransforms;

    mState mSTATE;
    mTarget mTARGET;

    ros::NodeHandle mNH;
};



#endif