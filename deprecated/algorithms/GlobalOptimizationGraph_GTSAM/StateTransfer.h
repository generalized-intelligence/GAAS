#ifndef STATE_TRANSFER_H
#define STATE_TRANSFER_H
//TODO:整个修改成根据GPS-SLAM Matcher进行的逻辑。
//短期中断或方差较大：不认为是异常。
//长期中断：进行地图切分。

#include "GPS_SLAM_Matcher.h"
#include <glog/logging.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/slam/dataset.h> 

#include <opencv2/core/persistence.hpp>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "GPSExpand.h"
#include "CallbacksBufferBlock.h"
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "GOG_Frame.h"
#include "GPS_like.h"

using namespace std;
using namespace ygz;
using namespace gtsam;


class StateTransferManager
{
public:
    const int STATE_NO_GPS = 0;
    const int STATE_WITH_GPS = 1;
    const int STATE_INITIALIZING = 2;//只有位置,没有yaw.
    StateTransferManager(GPS_SLAM_MATCHER& matcher,cv::FileStorage& fSettings,NonlinearFactorGraph& graph,GPSExpand& gps_coord,CallbackBufferBlock<sensor_msgs::NavSatFix>* pGPS_buffer_in,CallbackBufferBlock<geometry_msgs::PoseStamped>* pSLAM_buffer_in)
    {
        pSettings = &fSettings;
        this->pgps_slam_matcher = &matcher;
        this->patience = (*pSettings)["GPS_RECATCH_PATIENCE"];
        this->pGraph = &graph;
        this->pGPS_coord = &gps_coord;
        this->pGPS_buffer = pGPS_buffer_in;
        this->pSLAM_buffer = pSLAM_buffer_in;
        LOG(INFO)<<"StateTransferManager Initiated!"<<endl;
    }
    void getNewestYawAndVarianceRad(double& newestRad_out,double& newestCoariance_out)
    {
        newestRad_out = this->segment_yaw_slam_to_gps_initial.back()
                                  + 0;
                                   //this->estimatedBias;//获取对应的值+偏置.(在slam里 减去这个偏置. 后面实现.)
        newestCoariance_out = this->newestCovariance;//this->segment_yaw_slam_to_gps_initial_variance.back();//获取对应的值
    }
    int updateSlam()//接收到SLAM消息时候调用.
    {
        if(this->pgps_slam_matcher->matchLen() == 0)
        {
            return STATE_NO_GPS;
        }
        auto last_match = this->pgps_slam_matcher->at(this->pgps_slam_matcher->matchLen()-1);
        if(1)//WTF??
        {
            if( this->pSLAM_buffer->queryLastMessageTime() - this-> pGPS_buffer->at(last_match.gps_index).header.stamp.toSec() > 4)//TODO:阈值移到配置文件中.
            {
                if(this->currentState == STATE_INITIALIZING)
                {
                    LOG(INFO)<<"Initiating Failed:Drop GPS for over 4 sec.Drop out from STATE_INITIALIZING to STATE_NO_GPS"<<endl;
                    LOG(INFO)<<"newstate == STATE_NO_GPS for dropout from previous state."<<endl;
                }
                if(this->currentState == STATE_WITH_GPS)
                {
                    LOG(INFO)<<"Drop out from STATE_WITH_GPS to STATE_NO_GPS:Drop GPS for over 4 sec."<<endl;
                    LOG(INFO)<<"newstate == STATE_NO_GPS for dropout from previous state."<<endl;
                }
                this->currentState = STATE_NO_GPS;
            }
        }
        return currentState;
    }
    int updateState(bool gps_valid,bool& gpsLoopMode_output)//GPS_Valid只检验GPS和SLAM时间戳 以及gps covariance合格与否
    //返回新状态,以及在gpsLoopMode_output中给出这次是否应该以回环形式处理.
    {//读取里面GPS消息的状态.
        //TODO:set this->newestVariance 以提供查询.
        gpsLoopMode_output = false;
        if(this->currentState==STATE_WITH_GPS)
        {
            if(!gps_valid)
            {
                gps_invalid_count +=1;
                const int INVALID_COUNT_THRES=(*pSettings)["GPS_RECATCH_PATIENCE"];
                if(gps_invalid_count>INVALID_COUNT_THRES)
                {
                    this->currentState = STATE_NO_GPS;
                    this->gps_invalid_count = 0;
                    this->lastGPSSLAMMatchID = this->pgps_slam_matcher->matchLen()-1;
                }
            }
        }
        else if(this->currentState == STATE_INITIALIZING)
        {
            if(!gps_valid)
            {
                this->patience-=1;
            }
            else
            {
                bool init_success;
                int current_patience;
                double init_yaw,init_yaw_variance;
                checkMatcherToInitGPSYaw(init_success,init_yaw,init_yaw_variance);
                if(init_success)
                {
                    gpsLoopMode_output = true;
                    //processGPSRecatch(...);//重新匹配. //这段逻辑放到GlobalOptimizationGraph里.
                    currentState = STATE_WITH_GPS;
                    if(this->everInitWithGPS == false)
                    {
                        this->init_yaw_to_world_enu_rad_const = init_yaw;//只有这一次,初始化这个量.
                        LOG(INFO)<<"INIT YAW TO WORLD CONST_YAW = "<<init_yaw*180/3.1415926535<<"."<<endl;
                        this->everInitWithGPS = true;
                    }
                    //segment_beginning_match_id.push_back()
                    //segment_yaw_slam_to_gps_initial.push_back(xxx)
                    //gps_path_segment_id+=1;
                }
                else
                {
                    this->patience-=1;
                }
            }
            if(this->patience == 0)
            {//初始化失败了.
                this->patience = (*pSettings)["GPS_RECATCH_PATIENCE"];
                this->currentState = STATE_NO_GPS;
                this->segment_yaw_calib_beginning_id.pop_back();//反向消除改变.
            }
        }
        else if(this->currentState == STATE_NO_GPS)
        {
            if(gps_valid)
            {
                //尝试初始化yaw角度.将这个点作为第一个点.
                this->currentState = STATE_INITIALIZING;//然后有两种可能:初始化成功,或很快再次进入无GPS状态.
                segment_yaw_calib_beginning_id.push_back(pgps_slam_matcher->matchLen()-1);
            }
        }
        return this->currentState;
    }
    inline double getInitYawToWorldRad(bool& valueValid)
    {
        valueValid = this->everInitWithGPS;
        return this->init_yaw_to_world_enu_rad_const;
    }
    void checkMatcherToInitGPSYaw(bool& init_success,double& init_yaw,double& init_yaw_variance);//初始化成功与否 剩余可重试次数.
//从STATE_INITIALIZING 进行状态转移.
    inline int getCurrentState()
    {
        return this->currentState;
    }
    /*
    inline int getCurrentSegmentID()
    {
        return this->gps_path_segment_id;
    }
    void processGPSRecatch(Graph *graph,int gps_match_id1,int gps_match_id2)//处理GPS重捕获和对应的问题.
    {
        //首先 GPS重捕获意味着之前SLAM已经在没有GPS的环境下运行了很长时间.
        //这意味着可能积累的yaw误差已经很大了.
        //应该重新估计yaw的误差,而不能简单的认为和以前一样.
        
        //假设发生这样一个过程.
        //
        //1.SLAM单独在无GPS环境运行一段时间.
        //2.来到室外,GPS初始化成功,在matcher和GlobalOptimizationGraph::addBlockGPS()里成功建立了坐标系(关于地球的)
        //3.又回到室内,并运行一段时间,SLAM积累了yaw的误差.
        //4.来到室外.此时GPS重新初始化成功.应该做什么?
        //分析: 1.坐标系统有两套.基于地球的,GPS坐标系是没有累计误差的,所有优化应该以这个为前提.
        //      2.应该产生"回环",并认为初始的飞机朝向是固定的.一切误差在这个固定的偏航角基础上累计.
        //      3.这个"回环",不是立刻就能产生的.需要累计在再次回到室外后一段距离,以估计新的GPS-SLAM朝向角误差,由此推断SLAM在室内期间累计的误差.(认为短时间在室外飞行,产生的yaw误差可以忽略,具体实现通过标定精度优化实现.)
        //      4.修正之前所有的状态.虽然GPS不能直接测定偏航角,但可以通过一段时间和SLAM的偏航角差,确定这次回环的偏航角差.
        //      5.对于在(1)阶段累积的yaw误差,只能说仅仅通过GPS没有办法消除.
        for(int match_id = gps_match_id1,match_id<gps_match_id2;match_id++)
        {//加入从INITILIZING到WITH_GPS这一段的所有点进优化图.
            graph.emplace_back(match_id.slam_id,match_id.gps_id,Pose2(xxx,xxx,xxx));
        }
    }*/
    inline int getLastGPSSLAMMatchID()
    {
        return this->lastGPSSLAMMatchID;
    }
private:
    bool everInitWithGPS = false;
    int currentState = STATE_NO_GPS;
    int patience;
    double init_yaw_to_world_enu_rad_const = 0;
    int gps_path_segment_id = 0;//第几段有gps的轨迹.
    vector<double> segment_yaw_slam_to_gps_initial; // 每一段 初始化时候yaw init to gps.
                                                    //GlobalOptimizationGraph应该从这里读取这个角度.
                                                    //如果要加滤波器,应该要在这里加入.
    double newestCovariance;
    vector<int> segment_beginning_match_id;//每一段gps-slam都有的轨迹 开始时的match_id.可根据这个查询slam-gps的id和时间.
    vector<int> segment_yaw_calib_beginning_id;
    int lastGPSSLAMMatchID = -1;

    GPS_SLAM_MATCHER* pgps_slam_matcher;
    NonlinearFactorGraph* pGraph;
    GPSExpand* pGPS_coord;
    int gps_invalid_count = 0;
    cv::FileStorage* pSettings;
    CallbackBufferBlock<sensor_msgs::NavSatFix>* pGPS_buffer;
    CallbackBufferBlock<geometry_msgs::PoseStamped>* pSLAM_buffer;
};

void StateTransferManager::checkMatcherToInitGPSYaw(bool& init_success,double& init_yaw,double& init_yaw_variance)
//init_success:输出参数,这次初始化是否成功.
//init_yaw:输出参数.
//init_yaw_variance:输出参数.
{
//????这里先在哪里插入gps-slam对应关系??
    init_success = false;
    bool yaw_calc_valid = false;
    //double init_yaw_deg,init_yaw_deg_variance;

//取代原来GlobalOptimizationGraph中的实现.
//->
    if(this->pgps_slam_matcher->matchLen()>5)
    {
        bool yaw_calc_result_valid;
        double deg,deg_variance;
        pgps_slam_matcher->check2IndexAndCalcDeltaDeg(0,pgps_slam_matcher->matchLen()-1,//id
                                                       *pGPS_coord,yaw_calc_result_valid,deg,deg_variance
                                                        );//尝试计算yaw.
        
        if(yaw_calc_result_valid&& deg_variance < 20)//TODO:换成配置文件里的值.
        //满足variance<这个值 认为初始化成功了.
        {
            init_success = true;
            LOG(INFO)<<"YAW UPDATED.New value:"<<deg<<" deg,covariance:"<<deg_variance<<" deg."<<endl;
            double _rad = (deg*3.1415926535)/180;
            init_yaw = _rad;// init_yaw_deg*3.1415926/180;
            init_yaw_variance = deg_variance*3.1415926/180;
            //this->yaw_init_to_gps = fix_angle(_rad);
            this->segment_yaw_slam_to_gps_initial.push_back(_rad);
        }   
    }   
    else
    {
        LOG(INFO)<<"Not in check func.Len:"<<this->pgps_slam_matcher->matchLen()<<endl;
    }
//<-
}
















































/*
namespace StateTransfer
{
	const int state_NO_GPS_NO_SCENE = 0;
    const int state_NO_GPS_WITH_SCENE = 1;
    const int state_GPS_NO_SCENE = 2;
    const int state_GPS_WITH_SCENE = 3;

    bool GPS_avail(RIM* pRIM,double& new_lon_out,double& new_lat_out,double& new_alt_out)//check queue.
    {
        ...
    }
    void GPS_reinit(Graph* pGraph,coord& output_matching_coord)
    {
    	//SLAM coordinate will be matched to GPS ENU coordinate.
        auto coord = pGraph->getSLAMCoordinate();//x,y,z
        auto diff_coord = coord()
        double lon,lat,alt;
        if(GPS_avail(pRIM,lon,lat,alt))
        {
        	if(pRIM->hasGPSExpand())
        	{
        		//do optimization,connect edge GPS - NO_GPS - GPS.
        	}

        	pRIM->resetGPSExpand(lon,lat,alt);//重新寻找展开点.

        }
        

    }
}*/





#endif
