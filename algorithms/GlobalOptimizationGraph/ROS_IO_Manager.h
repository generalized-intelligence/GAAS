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
class ROS_IO_Manager
{
public:
    ROS_IO_Manager(int argc,char** argv);
    //just try init receiver.
    bool tryInitGPS();
    bool tryInitSLAM();
    //bool tryInitVelocity();
    bool doUpdateOptimizationGraph();
    bool publishAll();
    inline void setOptimizationGraph(shared_ptr<GlobalOptimizationGraph> pG)
    {
        this->pGraph = pG;
    }
private:
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


bool ROS_IO_Manager::tryInitGPS()//Just init receiver.Confirm message link status correct.
{
    if(this->SLAM_buffer.size()>this->pSettings["GPS_AVAIL_MINIMUM"])
    {
        return this->pGraph->init_gps();
    }
    return false;
}
bool ROS_IO_Manager::tryInitSLAM()
{
    bool result = false;
}



