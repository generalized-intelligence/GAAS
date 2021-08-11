#ifndef STATE_SURVEILLANCE_UTILS_H
#define STATE_SURVEILLANCE_UTILS_H

#include <iostream>
#include <sstream>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using std::cout;
using std::endl;
using std::stringstream;



void getEuclideanDistAndAngularDistFrom2Matrix4f(const Eigen::Matrix4f& m1,const Eigen::Matrix4f& m2,double& angular_dist_rad,double& euclidean_dist)
{
    //Eigen::Matrix4f dm = m1.inverse()*m2;
    Eigen::Matrix3f r1 = m1.block(0,0,3,3);
    Eigen::Matrix3f r2 = m2.block(0,0,3,3);

    Eigen::Quaternionf q1(r1);
    Eigen::Quaternionf q2(r2);
    angular_dist_rad = q1.angularDistance(q2);
    double dx = m1(0,3)-m2(0,3);
    double dy = m1(1,3)-m2(1,3);
    double dz = m1(2,3)-m2(2,3);
    euclidean_dist = sqrt(dx*dx + dy*dy + dz*dz);
}
Eigen::Matrix4f posestampedToEigenMatrix4f(const geometry_msgs::PoseStamped& pose)
{
    Eigen::Matrix4f mat;
    mat<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
    const auto& q = pose.pose.orientation;
    const auto& t = pose.pose.position;
    Eigen::Quaternionf q_(q.w,q.x,q.y,q.z);
    mat.block(0,0,3,3) = q_.toRotationMatrix();
    mat(0,3) = t.x;
    mat(1,3) = t.y;
    mat(2,3) = t.z;
    return mat;
}



#endif
