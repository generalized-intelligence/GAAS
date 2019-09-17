#ifndef RPY_QUAT_UTILS_H
#define RPY_QUAT_UTILS_H


#include <ros/ros.h>
#include <tf/transform_datatypes.h>
//#include <ar_track_alvar_msgs/AlvarMarkers.h>
//#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"

void R2ypr(Eigen::Matrix3d &R,double& y,double& p,double& r)
{
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        //Eigen::Vector3d ypr(3);
        y = atan2(n(1), n(0));
        p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
}


void getRPYFromQuat(double x,double y,double z,double w,double& roll,double& pitch,double& yaw)
{
    //tf::Quaternion quat(x,y,z,w);
    //double roll, pitch, yaw;
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Quaterniond quat(w,x,y,z);
    auto m = quat.toRotationMatrix();
    R2ypr(m,yaw,pitch,roll);
}


void getNewQuaternionFromOriginalQuaternionAndNewYawAngle(double x,double y,double z,double w,double new_yaw_rad,
                                                          double& newx,double& newy,double& newz,double& neww)
{
    double R,P,Y;//original RPY angles.
    getRPYFromQuat(x,y,z,w,R,P,Y);
    tf::Matrix3x3 newRmat;
    //newRmat.setRPY(R,P,new_yaw_rad);
    //newRmat.setRPY(R,new_yaw_rad,P);
    newRmat.setRPY(new_yaw_rad,P,Y);//maybe we should decompose the rotation matrix directly.
    tf::Quaternion quat_out_;
    newRmat.getRotation(quat_out_);
    newx = quat_out_.getX();
    newy = quat_out_.getY();
    newz = quat_out_.getZ();
    neww = quat_out_.getW();
}

#endif
