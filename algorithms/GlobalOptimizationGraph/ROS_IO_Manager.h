#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include "uNavAHRS.h"
#include <memory>

#include "CallbacksBufferBlock.h"

#include <sys/time.h>

#include <iostream>
using namespace std;
unsigned long long micros()//instead of micros in Arduino.h
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    unsigned long long result = (unsigned long long)(tp.tv_sec * 1000000 + tp.tv_usec);
    //std::cout<<"result:"<<result<<std::endl;
    return result;
}
typedef unsigned long long time_us_t;


class ROS_IO_Manager
{
public:
    ROS_IO_Manager(int argc,char** argv);
    //just try init receiver.
    //bool tryInitGPS();
    //bool tryInitSLAM();
    //bool tryInitVelocity();
    bool initOptimizationGraph();//gathered enough msgs in buffer.init estimator.
    bool doUpdateOptimizationGraph();//update OptimizationGraph once.
    bool publishAll();
    inline void setOptimizationGraph(shared_ptr<GlobalOptimizationGraph> pG)
    {
        this->pGraph = pG;
    }
    bool loopFunc()
    {
        ros::spinOnce();//Handle all callbacks.
        //一个简单实现:如果两种消息都凑齐至少一个,送一次.GPS有没有无所谓.
        if(this->SLAM_buffer.size()>0 && this->AHRS_buffer.size()>0)
        {
            pRIM->doUpdateOptimizationGraph();
            pRIM->publishAll();
        }
    }
private:
    time_us_t start_time_us;
    double ros_start_time;
    bool time_aligned = false;
    
    bool gps_avail = false;
  
    ros::NodeHandle *pNH;

    void AHRS_callback(const nav_msgs::Odometry& AHRS_msg);
    CallbacksBufferBlock<nav_msgs::Odometry> AHRS_buffer;
    ros::Subscriber AHRS_sub;

    void GPS_callback(const nav_msgs::NavSatFix& GPS_msg);
    CallbacksBufferBlock<nav_msgs::NavSatFix> GPS_buffer;
    ros::Subscriber GPS_sub;

    void SLAM_callback(const geometry_msgs::PoseStamped& SLAM_msg);
    CallbacksBufferBlock<geometry_msgs::PoseStamped> SLAM_buffer;
    ros::Subscriber SLAM_sub;

    shared_ptr<GlobalOptimizationGraph> pGraph;
    shared_ptr<cv::FileStorage> pSettings;

    //CallbacksBufferBlock<xxx_msgs::SceneRetrieveInfo> SceneRetrieve_Buffer;
    //ros::Subscriber SceneRetrieve_sub;
};


ROS_IO_Manager::ROS_IO_Manager(int argc,char** argv)
{
    //step<1> read config.
    this->pSettings (new cv::FileStorage (string(argv[1]),cv::FileStorage::READ));
    //step<2> init ros.
    ros::init(argc,argv);
    this->pNH = new ros::NodeHandle();
    //step<3> init subscriber.
    AHRS_sub = pNH->subscribe("/mavros/local_position/odom",10,boost::bind(this->AHRS_buffer.onCallbackBlock,_1)(&this->AHRS_buffer));
    GPS_sub = pNH->subscribe("/mavros/global_position/global",10,boost::bind(this->GPS_callback,_1)(this));
    SLAM_sub = pNH->subscribe("/SLAM/pose_for_obs_avoid",10,boost::bind(this->SLAM_callback,_1)(this));
    //SceneRetrieve_sub = pNH->subscribe("/..../,10,....")

}
bool ROS_IO_Manager::initOptimizationGraph()
{
    //step<1> init AHRS.Must success.
    bool ahrs_success = false;
    if( this->AHRS_buffer.size() > (*(this->pSettings))["AHRS_AVAIL_MINIMUM"])
    {
        ros::spinOnce(); //reduce timestamp error.
        this->start_time_us = micros();
        this->ros_start_time = this->AHRS_buffer.queryLastMessageTime();//align start time.
	this->time_aligned = true;
	ahrs_success = this->pGraph->init_AHRS();

    }
    if(!ahrs_success)
    {
        cout<<"Fatal error:AHRS init failed.Exit."<<endl;
        return false;
    }
    //step<2> check gps status.
    bool gps_success = false;
    if((*(this->pSettings))["ENABLE_GPS"])
    {
        //set init gps position.
        //we do not need spinOnce for we have to match timestamp with AHRS.
        double gps_init_time = this->GPS_buffer.queryLastMessageTime();
	if( abs(gps_init_time - this-ros_start_time) < (*(this->pSettings))["GPS_AHRS_MAX_TIME_DIFF_s"])
	{
	    gps_success = this->pGraph->init_gps();
	}	
    }
    if(!gps_success)
    {//still can we go on.set status first.
        cout<<"Warning:GPS init failed.Will continue."<<endl;
	this->gps_avail = false;	  
    }
    //step<3> check slam,match coordinates.Must success.
    bool slam_success = false;
    if(this->SLAM_buffer.size()>(*(this->pSettings))["SLAM_AVAIL_MINIMUM"])
    {
        double slam_init_time = SLAM_buffer.queryLastMessageTime();
        if(abs(slam_init_time-this->ros_start_time)< (*(this->pSettings))["SLAM_AHRS_MAX_TIME_DIFF_s"])
	{
            slam_success = this->pGraph->init_SLAM();
	}
    }
    if(!slam_success)
    {
        cout<<"Fatal error:SLAM init faied.Exit."<<endl;
	return false;
    }
    cout<<"OptimizationGraph init success!"<<endl;
    ros::spinOnce();//get latest msg.
    this->pGraph->tryInitVelocity();
    return true;
}
bool ROS_IO_Manager::doUpdateOptimizationGraph()
{
    //here we do update the graph.ros::spinOnce has been called.
    //check if we have necessary msgs.
    if(this->AHRS_buffer.size()>=1 && this->SLAM_buffer.size()>=1)
    {
        if(this->GPS_buffer.size()>=1)
        {
            this->pGraph->addBlockGPS(this->GPS_buffer.getLastMessage());//do update.
        }
        this->pGraph->addBlockAHRS(this->AHRS_buffer.getLastMessage());
        this->pGraph->addBlockSLAM(this->SLAM_buffer.getLastMessage());
    }


}

void ROS_IO_Manager::GPS_callback(const nav_msgs::NavSatFix& GPS_msg)
{
    this->GPS_buffer.onCallbackBlock(GPS_msg);
    //Do other callback procedure.
}
void ROS_IO_Manager::SLAM_callback(const geometry_msgs::PoseStamped& SLAM_msg)
{
    this->SLAM_buffer.onCallbackBlock(SLAM_msg);
    //Do other callback procedure.
}
bool ROS_IO_Manager::publishAll()
{
    auto pose = this->pGraph->pCurrentPR->esitmate();
    //make a ros msg.
}
/*
bool ROS_IO_Manager::tryInitGPS()//Just init receiver.Confirm message link status correct.
{
    if(this->SLAM_buffer.size()>this->pSettings["GPS_AVAIL_MINIMUM"])
    {
        this->gps_avail = true;
        return this->pGraph->init_gps();
    }
    return false;
}
bool ROS_IO_Manager::tryInitSLAM()
{
    bool result = false;
}*/



