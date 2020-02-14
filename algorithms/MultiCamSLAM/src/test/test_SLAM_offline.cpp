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

//离线测试SLAM系统.
class FileReaderWriter
{
    int frame_id = 0;
    int cam_count = 2;

    string basic_path = "/home/gi/GAAS/algorithms/data/01_bag/";
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
    vector<shared_ptr<cv::Mat> > getNextImageSet()
    {
        vector<shared_ptr<cv::Mat> > imgs;
        for(int cam_id =0;cam_id<cam_count;cam_id++)
        {
            std::string l = this->genearte_path_from_frame_id_cam_id_and_lr(this->frame_id,cam_id,true);
            std::string r = this->genearte_path_from_frame_id_cam_id_and_lr(this->frame_id,cam_id,false);
            shared_ptr<cv::Mat> l_im(new cv::Mat(cv::imread(l)));
            shared_ptr<cv::Mat> r_im(new cv::Mat(cv::imread(r)));
            imgs.push_back(l_im);
            imgs.push_back(r_im);
        }
        this->frame_id++;
        return imgs;
    }
    void read_iter()
    {
        auto imgs = getNextImageSet();
        pSLAM->iterateWith4Imgs(imgs[0],imgs[1],imgs[2],imgs[3]);//这个函数设计的是真的烂...
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


void FetchImageCallback(const sensor_msgs::ImageConstPtr& img1,const sensor_msgs::ImageConstPtr& img2,const sensor_msgs::ImageConstPtr& img3,const sensor_msgs::ImageConstPtr& img4)
{
    LOG(INFO)<<"In Fetch Image Callback:"<<endl;
    cv_bridge::CvImageConstPtr p1,p2,p3,p4;
    shared_ptr<cv::Mat> m1,m2,m3,m4;
    try
    {
        p1 = cv_bridge::toCvShare(img1);
        p2 = cv_bridge::toCvShare(img2);
        p3 = cv_bridge::toCvShare(img3);
        p4 = cv_bridge::toCvShare(img4);
        m1 = shared_ptr<cv::Mat> (new cv::Mat(p1->image.clone()));
        m2 = shared_ptr<cv::Mat> (new cv::Mat(p2->image.clone()));
        m3 = shared_ptr<cv::Mat> (new cv::Mat(p3->image.clone()));
        m4 = shared_ptr<cv::Mat> (new cv::Mat(p4->image.clone()));
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        LOG(ERROR)<<"cv_bridge exception: %s"<<e.what()<<endl;
        cout<<"cv_bridge exception: %s"<<e.what()<<endl;
        return;
    }
    LOG(INFO)<<"Images caught! Will call iterateWith4Imgs."<<endl;

    //pSLAM->iterateWith4Imgs(m1,m2,m3,m4);

    //write:
//    auto t1 = std::make_pair(m1,m2);
//    auto t2 = std::make_pair(m3,m4);
//    vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > v_;
//    v_.push_back(t1);v_.push_back(t2);
//    pReaderWriter->write_iter(v_);



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
    message_filters::Subscriber<sensor_msgs::Image> down_left_sub(nh, left_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> down_right_sub(nh, left_right_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_left_sub, front_right_sub,down_left_sub,down_right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4));
    ros::Subscriber sub = nh.subscribe("/mavros/imu/data_raw",100,FetchIMUCallBack);
    pSLAM = new SLAM_simple(argc,argv);
    pReaderWriter = new FileReaderWriter();
    while(true)
    {
        //read:
        pReaderWriter->read_iter();
    }
    ros::spin();
}

