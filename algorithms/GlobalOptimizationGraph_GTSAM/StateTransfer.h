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

    void resetState()
    {
        this->currentState = STATE_NO_GPS;
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
        LOG(INFO)<<"updateSlam: 1"<<endl;
        if(this->pgps_slam_matcher->matchLen() == 0)
        {
            return STATE_NO_GPS;
        }

        auto last_match = this->pgps_slam_matcher->at(this->pgps_slam_matcher->matchLen()-1);

        if( this->pSLAM_buffer->queryLastMessageTime() - this-> pGPS_buffer->at(last_match.gps_index).header.stamp.toSec() > 4)//TODO:阈值移到配置文件中.
        {
            LOG(INFO)<<"updateSlam 2, currentState: "<<currentState<<endl;
            if(this->currentState == STATE_INITIALIZING)
            {
                LOG(INFO)<<"Initiating Failed:Drop GPS for over 4 sec.Drop out from STATE_INITIALIZING to STATE_NO_GPS"<<endl;
                LOG(INFO)<<"newstate == STATE_NO_GPS for dropout from previous state."<<endl;
                return this->currentState;
            }

            if(this->currentState == STATE_WITH_GPS)
            {
                LOG(INFO)<<"Drop out from STATE_WITH_GPS to STATE_NO_GPS:Drop GPS for over 4 sec."<<endl;
                LOG(INFO)<<"newstate == STATE_NO_GPS for dropout from previous state."<<endl;
                return this->currentState;
            }
            this->currentState = STATE_NO_GPS;
        }

        LOG(INFO)<<"updateSlam 3, currentState: "<<currentState<<endl;
        return currentState;
    }

    //GPS_Valid只检验GPS和SLAM时间戳 以及gps covariance合格与否
    //返回新状态,以及在gpsLoopMode_output中给出这次是否应该以回环形式处理.
    int updateState(bool gps_valid, bool& gpsLoopMode_output)
    {   //读取里面GPS消息的状态.

        LOG(INFO)<<"gps_valid: "<<gps_valid<<", this->currentState: "<<this->currentState<<endl;

        //TODO:set this->newestVariance 以提供查询.
        gpsLoopMode_output = false;
        if(this->currentState==STATE_WITH_GPS)
        {
            LOG(INFO)<<"updateState STATE_WITH_GPS"<<endl;
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
            LOG(INFO)<<"updateState STATE_INITIALIZING"<<endl;
            if(!gps_valid)
            {
                LOG(INFO)<<"updateState not gps_valid"<<endl;
                this->patience-=1;
            }
            else
            {
                bool init_success;
                int current_patience;
                double init_yaw;
                double init_yaw_variance;
                checkMatcherToInitGPSYaw(init_success,init_yaw,init_yaw_variance);
                if(init_success)
                {
                    LOG(INFO)<<"updateState init succeed"<<endl;
                    gpsLoopMode_output = true;
                    //processGPSRecatch(...);//重新匹配. //这段逻辑放到GlobalOptimizationGraph里.
                    currentState = STATE_WITH_GPS;
                    if(this->everInitWithGPS == false)
                    {
                        // this global variable is used to map GPS dx and dy to SLAM coordinate
                        this->init_yaw_to_world_enu_rad_const = init_yaw;
                        LOG(INFO)<<"INIT YAW TO WORLD CONST_YAW = "<<init_yaw*180/3.1415926535<<"."<<endl;
                        this->everInitWithGPS = true;
                    }
                    //segment_beginning_match_id.push_back()
                    //segment_yaw_slam_to_gps_initial.push_back(xxx)
                    //gps_path_segment_id+=1;
                }
                else
                {
                    LOG(INFO)<<"updateState init failed!"<<endl;
                    this->patience-=1;
                }
            }

            if(this->patience == 0)
            {
                //init failed
                this->patience = (*pSettings)["GPS_RECATCH_PATIENCE"];
                this->currentState = STATE_NO_GPS;
                this->segment_yaw_calib_beginning_id.pop_back();//反向消除改变.
            }
        }
        else if(this->currentState == STATE_NO_GPS)
        {
            LOG(INFO)<<"updateState STATE_NO_GPS"<<endl;
            if(gps_valid)
            {
                // try to initialize YAW, set current idx as the beginning idx.
                // There could be two results:
                // 1. initialization succeed
                // 2. initialization failed and return to STATE_NO_GPS
                this->currentState = STATE_INITIALIZING;
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

    inline void set_last_slam_yaw_correction_id(int newest_yaw_correction_slam_frame_id, int newest_match_id)
    //在Global Optimization Graph中更新成功后,手动更新下这两个东西.
    {
        if(this->last_yaw_correction_slam_frame_id>=newest_yaw_correction_slam_frame_id)
        {
            LOG(ERROR)<<"ERROR:last_yaw_correction_slam_frame_id>=newest_yaw_correction_slam_frame_id!!! value:"<<last_yaw_correction_slam_frame_id<<","<<newest_yaw_correction_slam_frame_id<<"!!! CHECK YOUR INPUT IN GOG!!!!"<<endl;
            return;
        }
        this->last_yaw_correction_slam_frame_id = newest_yaw_correction_slam_frame_id;
        this->last_correction_GPS_SLAM_MATCH_ID = newest_match_id;
    }

    inline void get_last_yaw_correction_slam_id(int& last_slam_id_out,int& last_match_id_out)
    {
        last_slam_id_out = this->last_yaw_correction_slam_frame_id;
        last_match_id_out = this->last_correction_GPS_SLAM_MATCH_ID;
    }

    inline bool get_should_update_yaw_correction(int current_slam_frame_id)
    {//用一个简单策略:如果隔了100帧没更新过,那就挪到这个里面.
        if(this->last_yaw_correction_slam_frame_id<=0 || this->last_correction_GPS_SLAM_MATCH_ID<=0)
        {//尚未初始化.不管.
            return false;
        }
        if(current_slam_frame_id - this->last_yaw_correction_slam_frame_id>100) //TODO:挪到配置文件里面去.
        {
            return true;
        }
        else
        {
            return false;
        }
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

    int last_yaw_correction_slam_frame_id = -1;//最后一次匹配yaw的slamid.(后面的那个id.)
    int last_correction_GPS_SLAM_MATCH_ID = -1;//这个用于修正yaw累计误差,而不是初始化.和上面那个没关系.



    GPS_SLAM_MATCHER* pgps_slam_matcher;
    NonlinearFactorGraph* pGraph;
    GPSExpand* pGPS_coord;
    int gps_invalid_count = 0;
    cv::FileStorage* pSettings;
    CallbackBufferBlock<sensor_msgs::NavSatFix>* pGPS_buffer;
    CallbackBufferBlock<geometry_msgs::PoseStamped>* pSLAM_buffer;
};

void StateTransferManager::checkMatcherToInitGPSYaw(bool& init_success, double& init_yaw, double& init_yaw_variance)
{
    // try to change STATE FROM GPS_INIT TO WIT_GPS
    // init_success: output_param, if successfully initialized
    // init_yaw: output_param
    // init_yaw_variance: output_param
    // last_slam_frame_id: output_param, specify which slam frame the last update happened at.

    init_success = false;
    bool yaw_calc_valid = false;

    if(this->pgps_slam_matcher->matchLen()>5)
    {
        bool yaw_calc_result_valid;
        double deg, deg_variance;
        int match_index = pgps_slam_matcher->matchLen()-1;
        LOG(INFO)<<"checkMatcherToInitGPSYaw."<<endl;
//        pgps_slam_matcher->check2IndexAndCalcDeltaDegInit(0, match_index,*pGPS_coord,
//                                                          yaw_calc_result_valid,deg,deg_variance);

        pgps_slam_matcher->check2IndexAndCalcDeltaDeg(0, match_index,*pGPS_coord,
                                                          yaw_calc_result_valid,deg,deg_variance);

        //if variance is small than a threshold, we think it succeeded.
        if(yaw_calc_result_valid && deg_variance < 20)//TODO:换成配置文件里的值.
        {
            init_success = true;
            LOG(INFO)<<"YAW UPDATED. New value:"<<deg<<" deg, covariance:"<<deg_variance<<" deg."<<endl;
            double _rad = (deg*3.1415926535)/180;
            init_yaw = _rad;// init_yaw_deg*3.1415926/180;
            init_yaw_variance = deg_variance*3.1415926/180;
            //this->yaw_init_to_gps = fix_angle(_rad);
            this->segment_yaw_slam_to_gps_initial.push_back(_rad);

            this->last_yaw_correction_slam_frame_id = pgps_slam_matcher->queryMatchByIndex(match_index).slam_index;
            this->last_correction_GPS_SLAM_MATCH_ID = match_index;
        }   
    }   
    else
    {
        LOG(INFO)<<"Not in check func.Len:"<<this->pgps_slam_matcher->matchLen()<<endl;
    }

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
