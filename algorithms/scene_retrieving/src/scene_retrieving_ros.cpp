//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.
#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>

#include <image_process.h>

using namespace std;

std::shared_ptr<SceneRetriever> pSceneRetriever;

ros::Publisher Pub;

void ImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    cv::Mat curLeftImage, curRightImage;

    try
    {
        curLeftImage = cv_bridge::toCvShare(msgLeft)->image;
        curRightImage = cv_bridge::toCvShare(msgRight)->image;
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (curLeftImage.empty() && curRightImage.empty())
    {
        return;
    }


    cv::Mat RT_mat, Q_mat;
    bool match_success;
    int inliers = pSceneRetriever->retrieveSceneFromStereoImage(curLeftImage, curRightImage, Q_mat, RT_mat, match_success);

    if(match_success)
    {
        cout<<" Match success! RT mat is: \n"<<RT_mat<<endl;
        //TODO:publish RT mat!
        std_msgs::String str;
        stringstream ss;
        std::string str_content;
        ss<<"Frame id:"<<0<<","<<1<<";RT:"<<RT_mat;
        ss>>str_content;
        str.data = str_content.c_str();
        Pub.publish(str);
    }
    else
    {
        return;
    }



}


int main(int argc,char** argv)
{

    if (argc!=5)
    {
        cout<<"Usage: demo [scene_file_path] [voc_file_path] [l_image_path] [r_image_path] [Q_mat_file_path]"<<endl;
    }


    std::string scene_path(argv[1]), voc_file_path(argv[2]) , l_img_path(argv[3]), r_img_path(argv[4]), Q_mat_path(argv[5]);

    cout<<"scene path: "<<scene_path<<endl;
    cout<<"voc_file_path: "<<voc_file_path<<endl;
    cout<<"l_img_path: "<<l_img_path<<endl;
    cout<<"r_img_path: "<<r_img_path<<endl;
    cout<<"Q_mat_path: "<<Q_mat_path<<endl;

    cv::FileStorage fsSettings(Q_mat_path, cv::FileStorage::READ);
    cv::Mat Q_mat;
    fsSettings["Q_mat"] >> Q_mat;

    if (Q_mat.empty())
    {
        cout<<"Q mat empty, exit."<<endl;
        return -1;
    }


    cout<<"Q_mat: "<<endl<<Q_mat<<endl;

    cv::Mat RT_mat = (cv::Mat_<float >(4,4) << 1, 0, 0, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);



    //ros init
    ros::init(argc, argv, "scene_retrieve");
    ros::NodeHandle nh;


    Pub = nh.advertise<std_msgs::String>("/gaas/scene_retrieving",10);
    std::shared_ptr<SceneRetriever> pSceneRetrieve(new SceneRetriever(voc_file_path, scene_path));
    pSceneRetriever = pSceneRetrieve;


    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/gi/simulation/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/gi/simulation/right/image_raw", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(ImageCallback, _1, _2));



    return 0;
}








