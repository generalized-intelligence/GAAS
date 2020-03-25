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

#include "ROSPublisher.h"
#include <iostream>


using namespace std;





#define READ_MODE //读并离线运行 / 写入包
SLAM_simple* pSLAM;
mcs::ROSPublisher* pPublisher;
const bool USE_IMU = true;


//离线测试SLAM系统.
class FileReaderWriter
{
    int frame_id = 0;
    //int frame_id = 220;//避免下面看不见!

    int cam_count = 2;
    //int cam_count = 2;
    //int cam_count = 1;

    //string basic_path = "/home/gi/GAAS/algorithms/data/01_bag/";
    //string basic_path = "/home/gi/GAAS/algorithms/data/03_bag_5_aspect/";
    //string basic_path = "/home/gi/GAAS/algorithms/data/04_downonly/";
    string basic_path = "/home/gi/GAAS/algorithms/data/02_bag_5_aspect/";
    ofstream *os = nullptr;
    ifstream *is = nullptr;
    shared_ptr<sensor_msgs::Imu> pmsg_buf = nullptr;

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
//        this->frame_id++;//DEBUG ONLY.
//        this->frame_id++;
        return ret;//imgs;
    }
    bool parse_imu_line(ifstream& input,sensor_msgs::Imu& imu_msg_output,int &index_out)
    {
        if(input.eof())//检查文件非空.
        {
            return false;
        }
        char line[1000];
        input.getline(line,1000);
        stringstream ss(line);
        double wx,wy,wz,ax,ay,az;
        int index;
        ss>>wx;
        ss>>wy;
        ss>>wz;

        ss>>ax;
        ss>>ay;
        ss>>az;

        ss>>index;

        auto& omega = imu_msg_output.angular_velocity;
        omega.x = wx;
        omega.y = wy;
        omega.z = wz;

        auto& accl = imu_msg_output.linear_acceleration;
        accl.x = ax;
        accl.y = ay;
        accl.z = az;

        index_out = index;
    }
    void read_iter()
    {
        auto imgs = getNextImageSet();
        //pSLAM->iterateWith4Imgs(imgs[0],imgs[1],imgs[2],imgs[3]);//这个函数设计的是真的烂...
        auto p = getNextImageSet();
        //读取imu.
        vector<sensor_msgs::Imu> imu_info_vec;//当前帧的imu info.
        if(pmsg_buf!= nullptr)
        {
            imu_info_vec.push_back(*pmsg_buf);//上一帧的缓冲.
        }
        if(USE_IMU)
        {
            if(is == nullptr)
            {
                is = new ifstream();
                is->open(basic_path+"/imu.txt");
            }
            pmsg_buf = make_shared<sensor_msgs::Imu>();
            int index;
            bool res = parse_imu_line(*is,*pmsg_buf,index);
            while(index < this->frame_id)//需要跳过无效imu信息.
            {
                parse_imu_line(*is,*pmsg_buf,index);
            }
            while(true)
            {
                LOG(WARNING)<<"imu index:"<<index<<",frame_id:"<<this->frame_id;
                if (!res||index > this->frame_id)
                {
                    //文件结束.
                    LOG(INFO)<<"In loop: index:"<<index<<","<<"frame_id:"<<frame_id<<endl;
                    break;
                }
                else
                {
                    imu_info_vec.push_back(*pmsg_buf);
                    res = parse_imu_line(*is,*pmsg_buf,index);
                }
            }
            LOG(INFO)<<"imu_vec_tmp.size():"<<imu_info_vec.size()<<endl;
            pSLAM->imu_vec_tmp = imu_info_vec;
            pSLAM->iterateWithImgs(p,true);
        }
        else
        {
            pSLAM->iterateWithImgs(p,false);
        }
        bool valid;
        auto pose = pSLAM->getLastFramePose(valid);
        if(valid)
        {
            pPublisher->publish_ros_odometry(pose);
        }
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
        //写入IMU信息.
        if(!os)
        {
            os = new ofstream();
            os->open(basic_path+"/imu.txt");
        }
        for(int i = 0;i<pSLAM->imu_vec_tmp.size();i++)//读写硬盘的流程,直接从temp开始操作.和读包要完全一样.
        {
            auto rec = pSLAM->imu_vec_tmp.at(i);
            (*os)<<rec.angular_velocity.x<<" "<<rec.angular_velocity.y<<" "<<rec.angular_velocity.z<<" "<<rec.linear_acceleration.x<<" "<<rec.linear_acceleration.y<<" "<<rec.linear_acceleration.z<<" "<<this->frame_id-1<<endl;
            cout<<rec.angular_velocity.x<<" "<<rec.angular_velocity.y<<" "<<rec.angular_velocity.z<<" "<<rec.linear_acceleration.x<<" "<<rec.linear_acceleration.y<<" "<<rec.linear_acceleration.z<<" "<<rec.header.stamp.toSec()<<endl;
        }
        pSLAM->imu_vec_tmp.clear();
        os->flush();
    }
};
FileReaderWriter* pReaderWriter;


void FetchImageCallback(const sensor_msgs::ImageConstPtr& img1,const sensor_msgs::ImageConstPtr& img2,
                        const sensor_msgs::ImageConstPtr& img3,const sensor_msgs::ImageConstPtr& img4)
                        //,const sensor_msgs::ImageConstPtr& img5,const sensor_msgs::ImageConstPtr& img6) // 3组
{
    LOG(INFO)<<"In Fetch Image Callback:"<<endl;
    cout<<"In Fetch Image Callback:"<<endl;
    cv_bridge::CvImageConstPtr p1,p2,p3,p4;//,p5,p6;
    shared_ptr<cv::Mat> m1,m2,m3,m4;//,m5,m6;
    try
    {
        p1 = cv_bridge::toCvShare(img1);
        p2 = cv_bridge::toCvShare(img2);
        p3 = cv_bridge::toCvShare(img3);
        p4 = cv_bridge::toCvShare(img4);

        //p5 = cv_bridge::toCvShare(img5);
        //p6 = cv_bridge::toCvShare(img6);

        m1 = shared_ptr<cv::Mat> (new cv::Mat(p1->image.clone()));
        m2 = shared_ptr<cv::Mat> (new cv::Mat(p2->image.clone()));
        m3 = shared_ptr<cv::Mat> (new cv::Mat(p3->image.clone()));
        m4 = shared_ptr<cv::Mat> (new cv::Mat(p4->image.clone()));

        //m5 = shared_ptr<cv::Mat> (new cv::Mat(p5->image.clone()));
        //m6 = shared_ptr<cv::Mat> (new cv::Mat(p6->image.clone()));

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

    //auto t3 = std::make_pair(m5,m6);//第三组

    shared_ptr<vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > > pv_(new vector<std::pair<shared_ptr<cv::Mat>,shared_ptr<cv::Mat> > > );
    pv_->push_back(t1);pv_->push_back(t2);

    //pv_->push_back(t3);
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
    std::string front_left_topic("/gi/forward/left/image_raw"),front_right_topic("/gi/forward/right/image_raw");//,left_left_topic("/gi/leftward/left/image_raw"),left_right_topic("/gi/leftward/right/image_raw");

    message_filters::Subscriber<sensor_msgs::Image> front_left_sub(nh, front_left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> front_right_sub(nh, front_right_topic, 10);
    //message_filters::Subscriber<sensor_msgs::Image> left_left_sub(nh, left_left_topic, 10);
    //message_filters::Subscriber<sensor_msgs::Image> left_right_sub(nh, left_right_topic, 10);

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
    //        sensor_msgs::Image,sensor_msgs::Image,
            sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_left_sub, front_right_sub,//left_left_sub,left_right_sub,
                                                 //right_left_sub,right_right_sub,back_left_sub,back_right_sub,
                                                 down_left_sub,down_right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    //sync.setMaxIntervalDuration(ros::Duration(0.1));



    sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4));
    //sync.registerCallback(boost::bind(FetchImageCallback, _1, _2,_3,_4,_5,_6));
    ros::Subscriber sub = nh.subscribe("/mavros/imu/data_raw",100,FetchIMUCallBack);
    pPublisher = new mcs::ROSPublisher(nh);
    pSLAM = new SLAM_simple(argc,argv);
    pReaderWriter = new FileReaderWriter();
#ifdef READ_MODE
    while(true)
    {
        //read:
        pReaderWriter->read_iter();
    }
#else
    cout<<"ROS node listening and writing bags to folder!"<<endl;
    ros::spin();
#endif
}

