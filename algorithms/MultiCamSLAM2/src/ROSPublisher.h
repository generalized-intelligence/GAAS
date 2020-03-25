#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <cmath>


namespace mcs{



using namespace Eigen;

class ROSPublisher{
private:
    ros::Publisher pub;
public:
    ROSPublisher(ros::NodeHandle& nh)
    {
        this->pub = nh.advertise<nav_msgs::Odometry>("mcs_pose_pub",5);
    }


    void publish_ros_odometry(std::pair<Matrix3d,Vector3d>& pose)
    {

        Matrix3d rotation = pose.first;
        Vector3d translation = pose.second;
        Eigen::Quaterniond quat(rotation);
        nav_msgs::Odometry odom = nav_msgs::Odometry();

        //odom.
        odom.header.frame_id = "map";
        odom.pose.pose.position.x = translation[0];
        odom.pose.pose.position.y = translation[1];
        odom.pose.pose.position.z = translation[2];

        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        this->pub.publish(odom);
    }


};


}
