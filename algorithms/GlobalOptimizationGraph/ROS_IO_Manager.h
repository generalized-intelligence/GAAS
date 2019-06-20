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
#include "CallbacksBufferBlock.h"

#include <sys/time.h>

#include <iostream>
#include <thread>
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
        this->pGraph->initBuffers(this->SLAM_buffer,this->GPS_buffer,this->AHRS_buffer);
    }
    bool loopFunc()//return true if updated.
    {
        ros::spinOnce();//Handle all callbacks.
        //一个简单实现:如果两种消息都凑齐至少一个,送一次.GPS有没有无所谓.
        //TODO:delete check gps in this ().
        bool msg_avail = this->SLAM_buffer.size()>0 && this->AHRS_buffer.size()>0&&this->GPS_buffer.size()>0;//cout<<"TODO:remove GPS here."<<endl;
        //bool msg_avail = this->SLAM_buffer.size()>0 && this->AHRS_buffer.size()>0;
        if(msg_avail)
        {
            bool result = this->doUpdateOptimizationGraph();
            if(result)
            {
                this->publishAll();
                //clear buffers.
                this->SLAM_buffer.clear();
                this->AHRS_buffer.clear();
                this->GPS_buffer.clear();
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

    shared_ptr<GlobalOptimizationGraph> pGraph = nullptr;
    shared_ptr<cv::FileStorage> pSettings;

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
    boost::function<void(const boost::shared_ptr<nav_msgs::Odometry const>&
		   )> ahrs_callback(boost::bind(&ahrs_buffer_helper,boost::ref(this->AHRS_buffer),_1 ));
    boost::function<void(const boost::shared_ptr<sensor_msgs::NavSatFix const>&
		   )> gps_callback(boost::bind(&gps_buffer_helper,boost::ref(this->GPS_buffer),_1));
    boost::function<void(const boost::shared_ptr<geometry_msgs::PoseStamped const>&
		   )> slam_callback(
                                     boost::bind(&slam_buffer_helper,this,boost::ref(this->SLAM_buffer),_1)
                           );

    string ahrs_topic_name = (*pSettings)["AHRS_topic_name"];
    string gps_topic_name = (*pSettings)["GPS_topic_name"];
    string slam_topic_name = (*pSettings)["SLAM_topic_name"];
    AHRS_sub = pNH->subscribe(ahrs_topic_name,10,ahrs_callback);
    GPS_sub = pNH->subscribe(gps_topic_name,10,gps_callback);
    SLAM_sub = pNH->subscribe(slam_topic_name,10,slam_callback);

    //SLAM_sub = pNH->subscribe("/vins_estimator/odometry",10,slam_callback);
    cout <<"callback function binding finished!"<<endl;
    //
    //
    //SceneRetrieve_sub = pNH->subscribe("/..../,10,....")

}
bool ROS_IO_Manager::initOptimizationGraph()
{
    //step<1> init AHRS.Must success.
    bool ahrs_success = false;
    int ahrs_avail_minimum = (*(this->pSettings))["AHRS_AVAIL_MINIMUM"];
    ros::spinOnce();
    cout<<"AHRS_buffer.size:"<<AHRS_buffer.size()<<endl;
    if( this->AHRS_buffer.size() > ahrs_avail_minimum)
    {
        ros::spinOnce(); //reduce timestamp error.
        cout<<"AHRS_buffer.size:"<<AHRS_buffer.size()<<endl;
        this->start_time_us = micros();

        //TODO: when running live,select next line.
        //this->ros_start_time = this->AHRS_buffer.queryLastMessageTime();//align start time.
        this->ros_start_time = ros::Time::now().toSec();
        this->time_aligned = true;
        ahrs_success = this->pGraph->init_AHRS(this->AHRS_buffer.getLastMessage());
    }
    if(!ahrs_success)
    {
        cout<<"Fatal error:AHRS init failed.Exit."<<endl;
        return false;
    }
    //step<2> check gps status.
    bool gps_success = string("true")==(*(this->pSettings))["ENABLE_GPS"] && (this->GPS_buffer.size()>0);
    if(gps_success)
    {
        cout<<"Trying to init gps in RIM::initOptimizationGraph()!"<<endl;
        //set init gps position.
        //we do not need spinOnce for we have to match timestamp with AHRS.
        

        //TODO: when running live,select next line.
        //double gps_init_time = this->GPS_buffer.queryLastMessageTime();
        double gps_init_time = ros::Time::now().toSec();


        double gps_ahrs_max_time_diff_s = (*(this->pSettings))["GPS_AHRS_MAX_TIME_DIFF_s"];
        if( abs(gps_init_time - this->ros_start_time) < gps_ahrs_max_time_diff_s)
        {
            gps_success = this->pGraph->init_gps();
        }
    }
    if(!gps_success)
    {//still can we go on.set status first.
        cout<<"Warning:GPS init failed.Will continue."<<endl;
        this->gps_avail = false;
        
        //TODO:delete this.debug only.
        //  /*
        cout<<"DEBUG ONLY:check GPS FAILED.EXIT!"<<endl;
        return false;
        //  */
        
        
    }
    //step<3> check slam,match coordinates.Must success.
    bool slam_success = false;
    int slam_avail_minimum = (*(this->pSettings))["SLAM_AVAIL_MINIMUM"];
    if(this->SLAM_buffer.size()>slam_avail_minimum)
    {

        //TODO: when running live,select next line.
        //double slam_init_time = SLAM_buffer.queryLastMessageTime();
        double slam_init_time = ros::Time::now().toSec();


        double slam_ahrs_max_time_diff_s = (*(this->pSettings))["SLAM_AHRS_MAX_TIME_DIFF_s"];
        if(abs(slam_init_time-this->ros_start_time)< slam_ahrs_max_time_diff_s)
        {
            slam_success = this->pGraph->init_SLAM();
        }
        else
        {
            cout<<"SLAM info delay to high.Delay_s:"<<slam_init_time-this->ros_start_time<<endl;
        }
    }
    else
    {
        cout<<"SLAM buffer size:"<<SLAM_buffer.size()<<endl;
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
    //return true if optimize success.
    bool retval = false;
    if(this->AHRS_buffer.size()>=1 && this->SLAM_buffer.size()>=1&&this->GPS_buffer.size()>=1)
    {
        //step<1> add vertex.
        this->pGraph->addGOGFrame();
        //step<2> add edges.
        if(this->GPS_buffer.size()>=1)
        {
            this->pGraph->addBlockGPS(this->GPS_buffer.getLastMessage());//do update.
        }
        this->pGraph->addBlockAHRS(this->AHRS_buffer.getLastMessage());
        cout <<"add ahrs finished!"<<endl;
        this->pGraph->addBlockSLAM(this->SLAM_buffer.getLastMessage());
        cout <<"add block slam finished!"<<endl;
        retval = this->pGraph->doOptimization();
        this->pGraph->updateSLAM_AHRS_relative_rotation_translation();
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



