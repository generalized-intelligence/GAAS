//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.

#include <glog/logging.h>
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
cv::Mat* pQ_mat;
LoopClosingManager* plcm;
ros::Publisher Pub;

void ImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    cv::Mat curLeftImage, curRightImage;

    try
    {
        //curLeftImage = cv_bridge::toCvShare(msgLeft)->image;
        //curRightImage = cv_bridge::toCvShare(msgRight)->image;
        //auto l_im& = cv_bridge::toCvShare(msgLeft)->image;
        
        cv::cvtColor(cv_bridge::toCvShare(msgLeft)->image, curLeftImage, CV_BGR2GRAY);
        cv::cvtColor(cv_bridge::toCvShare(msgRight)->image, curRightImage, CV_BGR2GRAY);
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
    LOG(INFO)<<"Forming empty rt mat."<<endl;
    cv::Mat r_mat = cv::Mat::eye(3,3,CV_32F);
    cv::Mat t_mat = cv::Mat::zeros(3,1,CV_32F);
    LOG(INFO)<<"Generating frame..."<<endl;
    bool success_gen = false;
    auto current_scene_frame = generateSceneFrameFromStereoImage(curLeftImage,curRightImage,r_mat,t_mat,*pQ_mat,*plcm,success_gen);
    if(!success_gen)
    {
        LOG(INFO)<<"    after generateSceneFrameFromStereoImage():generate failed!"<<endl;
        return;
    }
    LOG(INFO)<<"adding frame into scene..."<<endl;
    pSceneRetriever->addFrameToScene(std::get<0>(current_scene_frame),std::get<1>(current_scene_frame),std::get<2>(current_scene_frame),std::get<3>(current_scene_frame),std::get<4>(current_scene_frame));
    int scene_frame_count = pSceneRetriever->getScene().getImageCount();
    LOG(INFO)<<"In ImageCallback():adding frame to scene by stereo..."<<endl;
    bool match_success;
    cv::Mat RT_mat;
    //int inliers = pSceneRetriever->retrieveSceneFromStereoImage(curLeftImage, curRightImage, *pQ_mat, RT_mat, match_success);
    cv::Mat cam_matrix = (cv::Mat_<float >(3,3) << 376, 0, 376, 
            0, 376, 240, 
            0, 0, 1);
    int loop_id;
    int inliers = pSceneRetriever->retrieveSceneWithScaleFromMonoImage(curLeftImage,cam_matrix,RT_mat,match_success,&loop_id);

    if(match_success)
    {
        LOG(INFO)<<"In scene_retrieving_ros ImageCallback():Match success! RT mat is: \n"<<RT_mat<<endl;
        //TODO:publish RT mat!
        std_msgs::String str;
        stringstream ss;
        std::string str_content;
        ss<<"Frame id:"<<loop_id<<","<<scene_frame_count<<";RT:"<<RT_mat;
        //ss>>str_content;
        str_content = ss.str();
        str.data = str_content.c_str();
        Pub.publish(str);
    }
    else
    {
        LOG(INFO)<<"Match failed."<<endl;
        return;
    }



}


int main(int argc,char** argv)
{
    google::InitGoogleLogging(argv[0]);
    if (argc!=3)
    {
        //cout<<"Usage: demo [scene_file_path] [voc_file_path] [l_image_path] [r_image_path] [Q_mat_file_path]"<<endl;
        cout<<"Usage: demo [voc_file_path] [Q_mat_file_path]"<<endl;
    }
    std::string voc_file_path(argv[1]) ,Q_mat_path(argv[2]);
    LoopClosingManager lcm(voc_file_path);
    plcm = &lcm;


    cout<<"voc_file_path: "<<voc_file_path<<endl;
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
    pQ_mat = &Q_mat;

    cv::Mat RT_mat = (cv::Mat_<float >(4,4) << 1, 0, 0, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);



    //ros init
    ros::init(argc, argv, "scene_retrieve");
    ros::NodeHandle nh;


    Pub = nh.advertise<std_msgs::String>("/gaas/scene_retrieving",10);
    std::shared_ptr<SceneRetriever> pSceneRetrieve(new SceneRetriever(voc_file_path));
    pSceneRetrieve->getScene().setHasScale(true);
    pSceneRetriever = pSceneRetrieve;


    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/gi/simulation/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/gi/simulation/right/image_raw", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(ImageCallback, _1, _2));

    ros::spin();

    return 0;
}








