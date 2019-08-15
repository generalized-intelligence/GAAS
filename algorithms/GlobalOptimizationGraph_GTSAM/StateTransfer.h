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
    StateTransferManager(GPS_SLAM_MATCHER& matcher,cv::Settings& fSettings)
    {
        this->pgps_slam_manager = &matcher;
        patience = fSettings["GPS_RECATCH_PATIENCE"];
    }
    int updateState(bool gps_valid,int new_gps_id)//返回新状态.
    {//读取里面GPS消息的状态.
        if(this->currentState==STATE_WITH_GPS)
        {
            if(!gps_valid)
            {
                gps_invalid_count +=1;
                if(gps_invalid_count>INVALID_COUNT_THRES)
                {
                    this->currentState = STATE_NO_GPS;
                    this->gps_invalid_count = 0;
                }
            }
        }
        if(this->currentState == STATE_NO_GPS)
        {
            if(gps_valid)
            {
                //尝试初始化yaw角度.将这个点作为第一个点.
                this->currentState = STATE_INITIALIZING;//然后有两种可能:初始化成功,或很快再次进入无GPS状态.
                //segment_yaw_calib_beginning_id.push_back(...)
            }
        }
        if(this->currentState == STATE_INITIALIZING)
        {
            bool init_success;
            int patience;
            double init_yaw,init_yaw_variance;
            checkMatcherToInitGPSYaw(init_success,patience,init_yaw,init_yaw_variance);
            if(init_success)
            {
                //processGPSRecatch(...);//重新匹配.
                //currentState = STATE_WITH_GPS
                //segment_beginning_match_id.push_back()
                //segment_yaw_slam_to_gps_initial.push_back(xxx)
                //gps_path_segment_id+=1;
            }
        }
    }
    void checkMatcherToInitGPSYaw(bool& init_success,int& patience,double& init_yaw,double& init_yaw_variance);//初始化成功与否 剩余可重试次数.
    int getCurrentState()
    {
    }
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
    }
private:
    int currentState = STATE_NO_GPS;
    int patience;

    int gps_path_segment_id = 0;//第几段有gps的轨迹.
    vector<double> segment_yaw_slam_to_gps_initial; // 每一段 初始化时候yaw init to gps.
                                                    //GlobalOptimizationGraph应该从这里读取这个角度.
                                                    //如果要加滤波器,应该要在这里加入.
    vector<int> segment_beginning_match_id;//每一段gps-slam都有的轨迹 开始时的match_id.可根据这个查询slam-gps的id和时间.
    vector<int> segment_yaw_calib_beginning_id;
    GPS_SLAM_MATCHER* pgps_slam_matcher;
    int gps_invalid_count = 0;
};



















































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
