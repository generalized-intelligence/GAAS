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


//#include "uNavAHRS.h"
#include <memory>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>
#include "CallbacksBufferBlock.h"

#include <sys/time.h>

#include <iostream>
#include <thread>
#include <mutex>

#include <glog/logging.h>
using namespace std;
unsigned long long micros()//instead of micros in Arduino.h
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    unsigned long long result = (unsigned long long)(tp.tv_sec * 1000000 + tp.tv_usec);
    //cout<<"micros():"<<result<<endl;
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
        this->pGraph->initBuffers(this->SLAM_buffer,this->GPS_buffer,this->AHRS_buffer,this->Velocity_buffer);
    }
    bool _gps_pos_update = false;
    bool _gps_vel_update = false;
    bool _slam_msg_update = false;

    bool loopFunc()//return true if updated.
    {
        ros::spinOnce();//Handle all callbacks.
        //一个简单实现:如果两种消息都凑齐至少一个,送一次.GPS有没有无所谓.
        //TODO:delete check gps in this ().
        //策略修改：这里只要有消息就调用对应的回调函数。
        //bool msg_avail = this->SLAM_buffer.size()>0 && this->AHRS_buffer.size()>0&&this->GPS_buffer.size()>0;cout<<"TODO:remove GPS here."<<endl;
        //bool msg_avail = this->SLAM_buffer.size()>0 && this->AHRS_buffer.size()>0;
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
    Matrix3d SLAM_ROTATION_EIGEN;
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

    shared_ptr<GlobalOptimizationGraph> pGraph = nullptr;
    shared_ptr<cv::FileStorage> pSettings;

    ros::Publisher attitude_ahrs;
    ros::Publisher attitude_slam;
    int attitude_marker_id = 0;
    //CallbackBufferBlock<xxx_msgs::SceneRetrieveInfo> SceneRetrieve_Buffer;
    //ros::Subscriber SceneRetrieve_sub;
};


#include "ros_buffer_helper.h" //function helpers.
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
ROS_IO_Manager::ROS_IO_Manager(int argc,char** argv)
{
    //step<1> read config.
    this->pSettings = shared_ptr<cv::FileStorage>(new cv::FileStorage ());
    pSettings->open(string(argv[1]),cv::FileStorage::READ);
    (*pSettings)["SLAM_ROTATION_MAT"] >> this->SLAM_ROTATION;
    cv2eigen(this->SLAM_ROTATION,SLAM_ROTATION_EIGEN);
    //step<2> init ros.
    ros::init(argc,argv,"GlobalOptimizationGraph_ROSNode");
    this->pNH = new ros::NodeHandle();
    //step<3> init subscriber.
    //
    //auto f1 = 
    //boost::function<void (const nav_msgs::Odometry)> 
    
    //reference:
    //const boost::function<void(const boost::shared_ptr<M const> &)> callback
    //auto f2 = 
    attitude_ahrs = this->pNH->advertise<visualization_msgs::Marker>("attitude_ahrs",1);
    attitude_slam = this->pNH->advertise<visualization_msgs::Marker>("attitude_slam",1);

    boost::function<void(const boost::shared_ptr<nav_msgs::Odometry const>&
		   )> ahrs_callback(boost::bind(&ahrs_buffer_helper,this,boost::ref(this->AHRS_buffer),_1 ));
    boost::function<void(const boost::shared_ptr<sensor_msgs::NavSatFix const>&
		   )> gps_callback(boost::bind(&gps_buffer_helper,this,boost::ref(this->GPS_buffer),_1));
    boost::function<void(const boost::shared_ptr<geometry_msgs::PoseStamped const>&
		   )> slam_callback(
                                     boost::bind(&slam_buffer_helper,this,boost::ref(this->SLAM_buffer),_1)
                           );
    boost::function<void(const boost::shared_ptr<geometry_msgs::TwistStamped const>& 
                   )> velocity_callback( boost::bind(&velocity_buffer_helper,this,boost::ref(this->Velocity_buffer),_1) 
                           );
    AHRS_sub = pNH->subscribe("/mavros/local_position/odom",10,ahrs_callback);
    GPS_sub = pNH->subscribe("/mavros/global_position/raw/fix",10,gps_callback);
    //SLAM_sub = pNH->subscribe("/SLAM/pose_for_obs_avoid",10,slam_callback);
    SLAM_sub = pNH->subscribe("/SLAM/pose_for_obs_avoid",10,slam_callback);//("/SLAM/pose_for_obs_avoid",10,slam_callback);//("/gaas/slam/pose",10,slam_callback);
    Velocity_sub = pNH->subscribe("/mavros/global_position/raw/gps_vel",10,velocity_callback);
    

    cout <<"callback function binding finished!"<<endl;
    //SceneRetrieve_sub = pNH->subscribe("/..../,10,....")

}
bool ROS_IO_Manager::initOptimizationGraph()
{
    //策略更改：只要有消息，就立刻开始。
    return true;
}
bool ROS_IO_Manager::doUpdateOptimizationGraph()
{
    //here we do update the graph.ros::spinOnce has been called.
    //check if we have necessary msgs.
    //return true if optimize success.
    bool retval = false;
    {
        //step<1> add vertex.
        //this->pGraph->addGOGFrame();
        //step<2> add edges.
        if(this->_slam_msg_update)
        {
            this->pGraph->addBlockSLAM(this->SLAM_buffer.size()-1);
            this->_slam_msg_update = false;
        }

        if(this->_gps_pos_update)
        {
            this->pGraph->addBlockGPS(this->GPS_buffer.size()-1);   //do update.
            this->_gps_pos_update = false;
        }
        if(this->_gps_vel_update)
        {
            this->pGraph->addBlockVelocity(this->Velocity_buffer.size()-1);
            this->_gps_vel_update = false;
        }
        //do optimization.
        retval = this->pGraph->doOptimization();
        this->slam_buf_mutex.unlock();
        this->gps_vel_mutex.unlock();
        this->gps_pos_mutex.unlock();
        //this->pGraph->updateSLAM_AHRS_relative_rotation_translation();
    }
    return retval;

}

void ROS_IO_Manager::GPS_callback(const sensor_msgs::NavSatFix& GPS_msg)
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
    cout<<"WARNING: RIM::publishAll not implemented!"<<endl;//TODO:fill this.
    //auto pose = this->pGraph->getpCurrentPR()->estimate();
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



