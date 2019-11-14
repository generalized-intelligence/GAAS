#pragma once


#include "GlobalOptimizationGraph.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <roseus/StringStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>

#include <memory>
#include <Eigen/Geometry>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>
#include "CallbacksBufferBlock.h"

#include <sys/time.h>

#include <iostream>
#include <thread>
#include <mutex>

#include <glog/logging.h>


using namespace std;
unsigned long long micros()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    unsigned long long result = (unsigned long long)(tp.tv_sec * 1000000 + tp.tv_usec);
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
        this->pGraph->initBuffers(this->SLAM_buffer,this->GPS_buffer,this->AHRS_buffer,this->Velocity_buffer,this->Barometer_buffer);
    }
    bool _gps_pos_update = false;
    bool _gps_vel_update = false;
    bool _slam_msg_update = false;
    bool _barometer_msg_update = false;

    bool loopFunc()//return true if updated.
    {
        ros::spinOnce();//Handle all callbacks.
        //一个简单实现:如果两种消息都凑齐至少一个,送一次.GPS有没有无所谓.
        //TODO:delete check gps in this ().
        bool msg_avail = this->_slam_msg_update ||this->_gps_pos_update;
        if(msg_avail)
        {
            bool result = this->doUpdateOptimizationGraph();
            if(result)
            {
                this->publishAll();
                //clear buffers.
                //this->SLAM_buffer.clear();
                //this->AHRS_buffer.clear();
                //this->GPS_buffer.clear();
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));//sleep 1ms to avoid waste of CPU period.
        return false;
    }

    inline shared_ptr<GlobalOptimizationGraph> getGraph()
    {
        return this->pGraph;
    }

    void publish_ahrs_marker(const visualization_msgs::Marker& m)
    {
        this->attitude_ahrs.publish(m);
    }

    void publish_slam_marker(const visualization_msgs::Marker& m)
    {
        this->attitude_slam.publish(m);
    }

    std::mutex slam_buf_mutex;
    std::mutex gps_vel_mutex;
    std::mutex gps_pos_mutex;


    cv::Mat SLAM_ROTATION;//fsSettings["LEFT.R"] >> R_l;
    cv::Mat SLAM_ROT_AND_TRANS;
    Matrix3d SLAM_ROTATION_EIGEN;
    Matrix3d SLAM_ROT_AND_TRANS_EIGEN;
    bool invert_slam_z = false;

private:

    time_us_t start_time_us;
    double ros_start_time;
    bool time_aligned = false;
    
    bool gps_avail = false;
  
    ros::NodeHandle *pNH;

    void AHRS_callback(const nav_msgs::Odometry& AHRS_msg);
    CallbackBufferBlock<nav_msgs::Odometry> AHRS_buffer;
    ros::Subscriber AHRS_sub;

    void GPS_callback(const sensor_msgs::NavSatFix& GPS_msg);
    CallbackBufferBlock<sensor_msgs::NavSatFix> GPS_buffer;
    ros::Subscriber GPS_sub;

    void SLAM_callback(const geometry_msgs::PoseStamped& SLAM_msg);
    CallbackBufferBlock<geometry_msgs::PoseStamped> SLAM_buffer;
    ros::Subscriber SLAM_sub;

    CallbackBufferBlock<geometry_msgs::TwistStamped> Velocity_buffer;
    ros::Subscriber Velocity_sub;

    CallbackBufferBlock<sensor_msgs::FluidPressure> Barometer_buffer;
    ros::Subscriber Barometer_sub;

    void gt_callback(const nav_msgs::Odometry& msg);
    ros::Subscriber GroundTruth_sub;

    shared_ptr<GlobalOptimizationGraph> pGraph = nullptr;
    shared_ptr<cv::FileStorage> pSettings;

    ros::Publisher attitude_ahrs;
    ros::Publisher attitude_slam;
    ros::Publisher gog_odometry;
    ros::Publisher state_string_publisher;
    int attitude_marker_id = 0;
    //CallbackBufferBlock<xxx_msgs::SceneRetrieveInfo> SceneRetrieve_Buffer;
    //ros::Subscriber SceneRetrieve_sub;

    ofstream mOutputPositionFile;
};







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



