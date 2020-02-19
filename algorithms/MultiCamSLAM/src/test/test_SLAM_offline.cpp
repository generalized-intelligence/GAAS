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


#define READ_MODE //读并离线运行 / 写入包
SLAM_simple* pSLAM;

//离线测试SLAM系统.
class FileReaderWriter
{
    //int frame_id = 0;
    int frame_id = 80;//避免下面看不见!

    int cam_count = 3;

    //string basic_path = "/home/gi/GAAS/algorithms/data/01_bag/";
    string basic_path = "/home/gi/GAAS/algorithms/data/02_bag_5_aspect/";
    //示例: $basic_path/0/l/1.jpg $basic_path/0/r/1.jpg $basic_path/1/l/1.jpg $basic_path/1/r/1.jpg
public:
    std::string genearte_path_from_frame_id_cam_id_and_lr(int frame_id,int cam_id,bool isLeft)
    {
        std::string ret_val;
        std::stringstream ss;
        char lr;
        if(isLeft)
        {
            lr = 'l';
        }
        else
        {
            lr = 'r';
        }
        ss<<basic_path<<"/"<<cam_id<<"/"<<lr<<"/"<<frame_id<<".jpg";
        ss>>ret_val;
        return ret_val;
    }
    //vector<shared_ptr<cv::Mat> >
    shared_ptr<vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > >getNextImageSet()
    {
        //vector<shared_ptr<cv::Mat> > imgs;
        shared_ptr<vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > > ret (new vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > ());
        for(int cam_id =0;cam_id<cam_count;cam_id++)
        {
            std::string l = this->genearte_path_from_frame_id_cam_id_and_lr(this->frame_id,cam_id,true);
            std::string r = this->genearte_path_from_frame_id_cam_id_and_lr(this->frame_id,cam_id,false);
            shared_ptr<cv::Mat> l_im(new cv::Mat(cv::imread(l)));
            shared_ptr<cv::Mat> r_im(new cv::Mat(cv::imread(r)));
            auto img_pair = std::make_pair(l_im,r_im);
            ret->push_back(img_pair);

            //imgs.push_back(l_im);
            //imgs.push_back(r_im);
        }
        this->frame_id++;
        return ret;//imgs;
    }
    void read_iter()
    {
        auto imgs = getNextImageSet();
        //pSLAM->iterateWith4Imgs(imgs[0],imgs[1],imgs[2],imgs[3]);//这个函数设计的是真的烂...
        auto p = getNextImageSet();
        pSLAM->iterateWithImgs(p);
    }
    void write_iter(vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > imgs)
    {
        assert(imgs.size() == this->cam_count);
        for(int cam_id =0;cam_id<cam_count;cam_id++)
        {
            std::string l = this->genearte_path_from_frame_id_cam_id_and_lr(this->frame_id,cam_id,true);
            std::string r = this->genearte_path_from_frame_id_cam_id_and_lr(this->frame_id,cam_id,false);
            cout<<"im path:"<<l<<endl;
            //shared_ptr<cv::Mat> l_im(new cv::Mat(cv::imread(l)));
            //shared_ptr<cv::Mat> r_im(new cv::Mat(cv::imread(r)));

            cv::imwrite(l,*(imgs.at(cam_id).first));
            cv::imwrite(r,*(imgs.at(cam_id).second));
        }
        this->frame_id++;
    }
};
FileReaderWriter* pReaderWriter;


void FetchImageCallback(const sensor_msgs::ImageConstPtr& img1,const sensor_msgs::ImageConstPtr& img2,
                        const sensor_msgs::ImageConstPtr& img3,const sensor_msgs::ImageConstPtr& img4 //)
                        ,const sensor_msgs::ImageConstPtr& img5,const sensor_msgs::ImageConstPtr& img6) // 3组
{
    LOG(INFO)<<"In Fetch Image Callback:"<<endl;
    cv_bridge::CvImageConstPtr p1,p2,p3,p4,p5,p6;
    shared_ptr<cv::Mat> m1,m2,m3,m4,m5,m6;
    try
    {
        p1 = cv_bridge::toCvShare(img1);
        p2 = cv_bridge::toCvShare(img2);
        p3 = cv_bridge::toCvShare(img3);
        p4 = cv_bridge::toCvShare(img4);

        p5 = cv_bridge::toCvShare(img5);
        p6 = cv_bridge::toCvShare(img6);

        m1 = shared_ptr<cv::Mat> (new cv::Mat(p1->image.clone()));
        m2 = shared_ptr<cv::Mat> (new cv::Mat(p2->image.clone()));
        m3 = shared_ptr<cv::Mat> (new cv::Mat(p3->image.clone()));
        m4 = shared_ptr<cv::Mat> (new cv::Mat(p4->image.clone()));

        m5 = shared_ptr<cv::Mat> (new cv::Mat(p5->image.clone()));
        m6 = shared_ptr<cv::Mat> (new cv::Mat(p6->image.clone()));

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        LOG(ERROR)<<"cv_bridge exception: %s"<<e.what()<<endl;
        cout<<"cv_bridge exception: %s"<<e.what()<<endl;
        return;
    }
    LOG(INFO)<<"Images caught! Will call iterateWith4Imgs."<<endl;




    //write:
    auto t1 = std::make_pair(m1,m2);
    auto t2 = std::make_pair(m3,m4);

    auto t3 = std::make_pair(m5,m6);//第三组

    shared_ptr<vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > > pv_(new vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > );
    pv_->push_back(t1);pv_->push_back(t2);

    pv_->push_back(t3);
#ifndef READ_MODE
    pReaderWriter->write_iter(*pv_);
#else
    //pSLAM->iterateWith4Imgs(m1,m2,m3,m4);
    //pSLAM->iterateWithImgs(pv_);
#endif


    LOG(INFO)<<"iterateWith4Imgs() finished."<<endl;
}
void FetchIMUCallBack(const sensor_msgs::ImuConstPtr& imu)
{
    sensor_msgs::Imu imu_copy = *imu;
    pSLAM->addIMUInfo(imu_copy);
}

int main(int argc,char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc,argv,"test_VO_node");
    ros::NodeHandle nh;
    std::string front_left_topic("/gi/forward/left/image_raw"),front_right_topic("/gi/forward/right/image_raw"),left_left_topic("/gi/leftward/left/image_raw"),left_right_topic("/gi/leftward/right/image_raw");

    message_filters::Subscriber<sensor_msgs::Image> front_left_sub(nh, front_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> front_right_sub(nh, front_right_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> left_left_sub(nh, left_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> left_right_sub(nh, left_right_topic, 10);

    //如果是5组:
    //message_filters::Subscriber<sensor_msgs::Image> right_left_sub(nh, left_left_topic, 10);
    //message_filters::Subscriber<sensor_msgs::Image> right_right_sub(nh, left_right_topic, 10);

    //message_filters::Subscriber<sensor_msgs::Image> back_left_sub(nh, left_left_topic, 10);
    //message_filters::Subscriber<sensor_msgs::Image> back_right_sub(nh, left_right_topic, 10);

    message_filters::Subscriber<sensor_msgs::Image> down_left_sub(nh, "/gi/downward/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> down_right_sub(nh, "gi/downward/right/image_raw", 10);


    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    //message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_left_sub, front_right_sub,left_left_sub,left_right_sub);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
            //sensor_msgs::Image,sensor_msgs::Image,
            //sensor_msgs::Image,sensor_msgs::Image,
            sensor_msgs::Image,sensor_msgs::Image,
            sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_left_sub, front_right_sub,left_left_sub,left_right_sub,
                                                 //right_left_sub,right_right_sub,back_left_sub,back_right_sub,
                                                 down_left_sub,down_right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));



    //sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4));
    sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4,_5,_6));
    ros::Subscriber sub = nh.subscribe("/mavros/imu/data_raw",100,FetchIMUCallBack);
    pSLAM = new SLAM_simple(argc,argv);
    pReaderWriter = new FileReaderWriter();
#ifdef READ_MODE
    while(true)
    {
        //read:
        pReaderWriter->read_iter();
    }
#else
    ros::spin();
#endif
}

