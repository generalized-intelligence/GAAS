#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>

#include "ROS_Wrapper.h"









MeshGeneratorROSWrapper* pWrapper;


void FetchImgCallback(int cam_id,const sensor_msgs::ImageConstPtr& img1,const sensor_msgs::ImageConstPtr& img2,
                                    const sensor_msgs::ImageConstPtr& img3,const sensor_msgs::ImageConstPtr& img4
                      )
{
    ScopeTimer t("Time Cost of 2 img pairs:");
    cv::Mat p1,p2,p3,p4;//,p5,p6;
    shared_ptr<cvMatT> m1,m2,m3,m4;//,m5,m6;
    try
    {
        p1 = cv_bridge::toCvShare(img1)->image.clone();
        p2 = cv_bridge::toCvShare(img2)->image.clone();
        p3 = cv_bridge::toCvShare(img3)->image.clone();
        p4 = cv_bridge::toCvShare(img4)->image.clone();

        m1 = shared_ptr<cvMatT> ( new cvMatT(convertMatTocvMatT(p1)));
        m2 = shared_ptr<cvMatT> ( new cvMatT(convertMatTocvMatT(p2)));
        m3 = shared_ptr<cvMatT> ( new cvMatT(convertMatTocvMatT(p3)));
        m4 = shared_ptr<cvMatT> ( new cvMatT(convertMatTocvMatT(p4)));
    }
    catch (Exception e)
    {
        LOG(ERROR)<<e.what()<<endl;
    }
    pWrapper->onCallback(m1,m2,m3,m4,img1);
}





int main(int argc,char** argv)
{
//    assert (argc ==2);
//    cv::FileStorage fsettings;
//    fsettings.open(argv[1],cv::FileStorage::READ);
    google::InitGoogleLogging(argv[0]);
    ros::init(argc,argv,"MeshGeneratorNode");
    ros::NodeHandle nh;
    std::string front_left_topic("/gi/forward/left/image_raw"),front_right_topic("/gi/forward/right/image_raw");//,left_left_topic("/gi/leftward/left/image_raw"),left_right_topic("/gi/leftward/right/image_raw");

    message_filters::Subscriber<sensor_msgs::Image> front_left_sub(nh, front_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> front_right_sub(nh, front_right_topic, 10);

    message_filters::Subscriber<sensor_msgs::Image> down_left_sub(nh, "/gi/downward/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> down_right_sub(nh, "gi/downward/right/image_raw", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
            //sensor_msgs::Image,sensor_msgs::Image,
            //sensor_msgs::Image,sensor_msgs::Image,
            //        sensor_msgs::Image,sensor_msgs::Image,
            sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_left_sub, front_right_sub,//left_left_sub,left_right_sub,
                                                 //right_left_sub,right_right_sub,back_left_sub,back_right_sub,
                                                 down_left_sub,down_right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));


    int cam_id = 0;
    sync.registerCallback(boost::bind(FetchImgCallback,cam_id,_1, _2,_3,_4));
    //sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4,_5,_6));
    //ros::Subscriber sub = nh.subscribe("/mavros/imu/data_raw",100,FetchIMUCallBack);
    pWrapper = new MeshGeneratorROSWrapper(&nh);

    ros::spin();
}
