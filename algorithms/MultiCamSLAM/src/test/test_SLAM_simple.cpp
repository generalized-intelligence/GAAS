#include "Frame.h"
#include "FeatureFrontEndCV.h"
#include "FrameWiseGeometry.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h> //for DJI.


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h> //for apm and pixhawk.

#include <visualization_msgs/Marker.h> //for visualization.

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h> //for px4's external pose estimate
#include <sensor_msgs/Imu.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <Eigen/Core>
#include <Eigen/Dense>  // linear algebra
#include <Eigen/StdVector>




#include <deque>
#include <chrono>
#include "SLAM_simple.h"
using namespace std;



SLAM_simple* pSLAM;
void FetchImageCallback(const sensor_msgs::ImageConstPtr& img1,const sensor_msgs::ImageConstPtr& img2,const sensor_msgs::ImageConstPtr& img3,const sensor_msgs::ImageConstPtr& img4)
{
    cv_bridge::CvImageConstPtr p1,p2,p3,p4;
    shared_ptr<cv::Mat> m1,m2,m3,m4;
    try
    {
        p1 = cv_bridge::toCvShare(img1);
        p2 = cv_bridge::toCvShare(img2);
        p3 = cv_bridge::toCvShare(img3);
        p4 = cv_bridge::toCvShare(img4);
        m1 = shared_ptr<cv::Mat> (new cv::Mat(p1->image));
        m2 = shared_ptr<cv::Mat> (new cv::Mat(p2->image));
        m3 = shared_ptr<cv::Mat> (new cv::Mat(p3->image));
        m4 = shared_ptr<cv::Mat> (new cv::Mat(p4->image));
    }
    catch (cv_bridge::Exception& e)
    {
        LOG(ERROR)<<"cv_bridge exception: %s"<<e.what()<<endl;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    LOG(INFO)<<"Images caught!"<<endl;
    pSLAM->iterateWith4Imgs(m1,m2,m3,m4);
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_VO_node");
    ros::NodeHandle nh;
    std::string front_left_topic,front_right_topic,down_left_topic,down_right_topic;

    message_filters::Subscriber<sensor_msgs::Image> front_left_sub(nh, front_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> front_right_sub(nh, front_right_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> down_left_sub(nh, down_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> down_right_sub(nh, down_right_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_left_sub, front_right_sub,down_left_sub,down_right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4));
    pSLAM = new SLAM_simple(argc,argv);
    ros::spin();
}

