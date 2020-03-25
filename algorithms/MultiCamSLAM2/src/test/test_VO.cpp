//一个VO,简单实现基于optflow获取3d点,再用 solvepnp 获取更新位置.没有优化部分.

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
using namespace std;
class VO_simple
{
public:
    VO_simple(int argc,char** argv)
    {
        begin_t = std::chrono::high_resolution_clock::now();
    }
    bool needNewKeyFrame()
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        if (!ever_init || double(std::chrono::duration_cast<std::chrono::nanoseconds>(t_now-last_kf_update_t).count()/1e9) > 0.5)
        {//每0.5s,创建一次关键帧.
            last_kf_update_t = t_now;
            last_frame_update_t = t_now;
            //ever_init = true;
            return true;
        }
        return false;
    }
    shared_ptr<mcs::Frame> getLastKF()
    {
        return this->pLastKF;
    }
    shared_ptr<mcs::Frame> getLastFrame()
    {
        return this->pLastF;
    }
    void iterateWith4Imgs(shared_ptr<cv::Mat> img1,shared_ptr<cv::Mat> img2,shared_ptr<cv::Mat> img3,shared_ptr<cv::Mat> img4)
    {
        shared_ptr<mcs::Frame> pNewF;
        bool needNewKF;
        shared_ptr < vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > > > pvInputs( new vector<std::pair<shared_ptr<mcs::cvMat_T>,shared_ptr<mcs::cvMat_T> > >());
        pvInputs->push_back(std::make_pair(img1,img2));
        pvInputs->push_back(std::make_pair(img3,img4));
        if(needNewKeyFrame())
        {
            bool needNewKF = true;
            bool create_frame_success;
            pNewF = mcs::createFrameStereos(pvInputs,this->cam_config,create_frame_success,needNewKF);
            if(!ever_init)
            {
                LOG(INFO)<<"init VO!"<<endl;//初始化VO.
                pNewF->rotation = Eigen::Matrix3d::Identity();
                pNewF->position = Eigen::Vector3d(0,0,0);
                ever_init = true;
            }
            else
            {
                LOG(INFO)<<"VO initiated.Will trackLocalFrame()."<<endl;
                bool track_localframe_success;
                //mcs::trackLocalFramePoints(); // TODO:frame wise using p3d tracking;
                if(track_localframe_success)
                {
                    //pNewF->rotation = ...
                    //pNewF->position =
                    if(this->VORunningState != STATE_TRACKING)
                    {
                        LOG(INFO)<<"State transfer from "<<state_id_map[VORunningState]<<" to STATE_TRACKING!"<<endl;
                    }
                    this->VORunningState = this->STATE_TRACKING;
                }
                else
                {
                    LOG(ERROR)<<"Track failure!Using last frame rt."<<endl;
                    pNewF->rotation = getLastFrame()->rotation;
                    pNewF->position = getLastFrame()->position;
                    if(this->VORunningState!= this->STATE_TRACKING_FALIED)
                    {
                        LOG(WARNING)<<"State transfer from "<<state_id_map[VORunningState]<<" to STATE_TRACKING_FAILED!"<<endl;
                    }
                    this->VORunningState = this->STATE_TRACKING_FALIED;
                }
            }
            pLastKF = pNewF;
            pLastF = pNewF;
        }
        else
        {
            bool needNewKF =false;
            bool create_frame_success;
            last_frame_update_t = std::chrono::high_resolution_clock::now();
            pNewF = mcs::createFrameStereos(pvInputs,this->cam_config,create_frame_success,needNewKF);
            //TODO:
            bool track_and_pnp_ransac_success;
            //mcs::trackAndDoSolvePnPRansacMultiCam(pNewF); //frame_wise tracking....
            if(track_and_pnp_ransac_success)
            {
                //createNewKF...
            }
            else
            {
                //pNewF->rotation = ...
                //pNewF->translation = ...
            }
        }
    }
public:
    static const int STATE_TRACKING = 2;
    static const int STATE_UNSTABLE_TRACKING = 1;
    static const int STATE_TRACKING_FALIED = 0;
    map<int,std::string> state_id_map = {
                                            {0,"STATE_TRACKING"},
                                            {1,"STATE_UNSTABLE_TRACKING"},
                                            {2,"STATE_TRACKING_FAILED"}
                                     };
private:
    int VORunningState = STATE_TRACKING_FALIED; // init.
    int unstable_tracking_patience = 0;//暂时不用.用到的时候使用它计量已经有多少个不稳定追踪.
    deque<mcs::Frame> frameQueue;
    vector<StereoCamConfig> cam_config;
    shared_ptr<mcs::Frame> pLastKF=nullptr,pLastF=nullptr;
    std::chrono::high_resolution_clock::time_point begin_t;
    std::chrono::high_resolution_clock::time_point last_kf_update_t;
    std::chrono::high_resolution_clock::time_point last_frame_update_t;
    bool ever_init = false;


};

VO_simple* pVO;
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
    pVO->iterateWith4Imgs(m1,m2,m3,m4);
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
    pVO = new VO_simple(argc,argv);
    ros::spin();
}













