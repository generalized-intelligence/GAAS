#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H
#define EIGEN_MAX_STATIC_ALIGN_BYTES 0  //To avoid Eigen memory alignment error.

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <glog/logging.h>

#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/slam/dataset.h> 
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <opencv2/core/persistence.hpp>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mutex>

#include "GPSExpand.h"
#include "CallbacksBufferBlock.h"
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "GOG_Frame.h"
#include "GPS_like.h"
#include "GPS_SLAM_Matcher.h"
#include "StateTransfer.h"

#include "modules/Barometer_module.h"
#include "modules/RPY_Quat_utils.h"
#include "modules/Optimizer_utils.h"
#include "modules/Loop_utils.h"

using namespace std;
using namespace gtsam;
const float thHuber = 50;

const int GPS_INIT_BUFFER_SIZE = 100;


const double EPS = 0.0001;
const double PI = 3.1415926535;

double fix_angle(double& angle)
{
    while(angle>2*PI)
    {
        angle-=2*PI;
    }
    while (angle<0)
    {
        angle+=2*PI;
    }
    double ret_val = angle;
    return ret_val;
}

double get_yaw_from_slam_msg(const geometry_msgs::PoseStamped &m);

void get_rpy_from_slam_msg(const geometry_msgs::PoseStamped &m,double& roll,double& pitch,double& yaw);

typedef struct OptimizationGraphStatus
{
    bool state_correct = false;
    Quaterniond ret_val_R;
    Vector3d ret_val_t;
    std_msgs::Header header_;
    int innerID_of_GOG;
}
OptimizationGraphStatusT;

class GlobalOptimizationGraph
{

public:

    GlobalOptimizationGraph(int argc,char** argv);
    
    bool checkAHRSValid();
    bool checkSLAMValid();
    bool checkQRValid();
    bool checkGPSValid(const sensor_msgs::NavSatFix& gps);
    
    bool init_AHRS(const nav_msgs::Odometry& AHRS_msg);
    bool init_gps();//init longitude,latitude,altitude.
    bool init_SLAM();//match Coordinate.
    //TODO:Init a GPS callback buffer block class,
    //inherit callback buffer base,implement init and check avail.
    bool inputGPS(const sensor_msgs::NavSatFix& gps);
    bool tryInitVelocity();//参考ygz的EdgePRV.
    void addBlockAHRS(const nav_msgs::Odometry& AHRS_msg);
    //SLAM msg as fixed vertexpr node.
    void addBlockSLAM(int msg_index);//(const geometry_msgs::PoseStamped& SLAM_msg);

    void addBlockGPS(int msg_index);//(const sensor_msgs::NavSatFix& GPS_msg);
    void addBlockBarometer(int msg_index);
    void addBlockVelocity(int msg_index);//(const geometry_msgs::TwistStamped& velocity_msg);
    void addBlockQRCode();
    void addBlockLoop(const LoopMessage& msg);
    void addBlockFCAttitude();
    //void addBlockSceneRetriever();
    
    //SLAM msg as edge prv and vertexspeed.
    std::deque<nav_msgs::Odometry> slam_prv_buffer;    
    void addSLAM_edgeprv(const geometry_msgs::PoseStamped& SLAM_msg);
    void DebugPrintWholeGraphValue();
    bool doOptimization();
    void resetOptimizationGraph()
    {
    }
    void initBuffers(CallbackBufferBlock<geometry_msgs::PoseStamped> & SLAMbuf,
                     CallbackBufferBlock<sensor_msgs::NavSatFix>& GPSbuf,
                     CallbackBufferBlock<nav_msgs::Odometry>& AHRSbuf,
                     CallbackBufferBlock<geometry_msgs::TwistStamped>& Velocitybuf,
                     CallbackBufferBlock<sensor_msgs::FluidPressure>& Barometerbuf
    )
    {
        this->pSLAM_Buffer = &SLAMbuf;
        this->pGPS_Buffer = &GPSbuf;
        this->pVelocity_Buffer = &Velocitybuf;
        this->pAHRS_Buffer = &AHRSbuf;
        this->pBarometer_Buffer = &Barometerbuf;
        this->p_gps_slam_matcher = shared_ptr<GPS_SLAM_MATCHER>(new GPS_SLAM_MATCHER(this->pSLAM_Buffer,this->pGPS_Buffer,&(this->fSettings )) );
        this->p_state_tranfer_manager = shared_ptr<StateTransferManager>(new StateTransferManager(*p_gps_slam_matcher,this->fSettings,this->graph,this->GPS_coord,this->pGPS_Buffer,this->pSLAM_Buffer));
        this->p_BarometerManager = shared_ptr<BarometerManager>(new BarometerManager());
    }

    //std::tuple<bool,Quaterniond,Vector3d,double,int> queryCurrentFullStatus()

    OptimizationGraphStatusT queryCurrentFullStatus()
    {
        state_mutex.lock();
        auto ret_val = this->current_status;
        state_mutex.unlock();
        return ret_val;
    }

private:

    cv::FileStorage fSettings;

public:

    static const int STATUS_NO_GPS_NO_SCENE = 0;
    static const int STATUS_NO_GPS_WITH_SCENE = 1;
    static const int STATUS_WITH_GPS_NO_SCENE = 2;
    static const int STATUS_GPS_SCENE = 3;

    int GPS_AVAIL_MINIMUM;

    inline int getStatus()
    {
        return this->status;
    }

private:

    shared_ptr<GPS_SLAM_MATCHER> p_gps_slam_matcher;
    shared_ptr<StateTransferManager> p_state_tranfer_manager;
    shared_ptr<BarometerManager> p_BarometerManager;

    NonlinearFactorGraph graph;
    Values initialEstimate;
    double yaw_rad_current_estimate = 0;
    const int relinearizeInterval = 2000;//300;
    //NonlinearISAM* p_isam;//(relinearizeInterval);
    //NonlinearISAM* p_isam;
    ISAM2* p_isam;
    bool online_mode = true;

    int status = STATUS_NO_GPS_NO_SCENE;

    int slam_vertex_index = 0;
    int last_gps_vertex_index = -1;
    int last_gps_vel_index = -1;
    int last_baro_vertex_index = -1;
    double yaw_init_to_slam = 0.0; //GOG初始的y轴 转向 slam的y轴夹角.

    double current_yaw_slam_drift = 0;
    //double yaw_init_to_gps = 0.0; //GOG的y轴 到 gps的北 夹角.
    
    //message buffers.
    
    CallbackBufferBlock<geometry_msgs::PoseStamped> * pSLAM_Buffer;
    CallbackBufferBlock<sensor_msgs::NavSatFix> * pGPS_Buffer;
    CallbackBufferBlock<geometry_msgs::TwistStamped> * pVelocity_Buffer;
    CallbackBufferBlock<nav_msgs::Odometry> * pAHRS_Buffer;
    CallbackBufferBlock<sensor_msgs::FluidPressure>* pBarometer_Buffer;
    //uav location attitude info management.
    void remap_UAV_coordinate_with_GPS_coordinate();

    bool gps_expand_ever_init = false;
    GPSExpand GPS_coord;

    bool allow_gps_usage = true;//can be configured by config.yaml
    double gps_init_longitude,gps_init_latitude,gps_init_altitude;

    double gps_init_lon_variance,gps_init_lat_variance,gps_init_alt_variance;
    std::vector<sensor_msgs::NavSatFix> gps_info_buffer;
    //time for calc speed vertex
    double last_slam_msg_time;
    double init_slam_msg_time;

    OptimizationGraphStatusT current_status;
    std::mutex state_mutex;

    ofstream mInputSlamFile;
};

GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{

    testRPY_INVERSE();
    testRPY_INVERSE(2,3,1);
    testRPY_INVERSE(3,1,2);

    //cv::FileStorage fSettings;//(string(argv[1]),cv::FileStorage::READ);
    this->fSettings.open(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];

    //Initialize optimizer:

    //p_isam = new NonlinearISAM (relinearizeInterval);
    ISAM2Params isam2_params_;
    int enable_relinearize_in = this->fSettings["ENABLE_RELINEARIZE"];
    bool enable_relinearize = (enable_relinearize_in!=0);
    double relinearizeThres = this->fSettings["RELINEARIZE_THRES"];
    isam2_params_.relinearizeThreshold = relinearizeThres; //0.01;
    int relinearizeSkip_num = this->fSettings["RELINEARIZE_SKIP_NUM"];
    isam2_params_.relinearizeSkip = relinearizeSkip_num;//1;//多少个变量之后,relinearize.
    isam2_params_.enableRelinearization = enable_relinearize;//false;//禁止relinearize.

    int online_mode_i = fSettings["ONLINE_MODE"];
    if(online_mode_i)
    {
        p_isam = new ISAM2(isam2_params_);
        this->online_mode = true;
    }
    else
    {
        this->online_mode = false;
    }

    mInputSlamFile.open("./results/slam_position.txt");
}

bool gps_msg_is_valid(const sensor_msgs::NavSatFix& gps)
{
    if(gps.status.status>=0 && gps.position_covariance_type>=2)
    {
        if( sqrt(pow(gps.position_covariance[0],2) + pow(gps.position_covariance[4],2)) < 30000)//<3
        {
            return true;
        }
    }
    return false;
}

void GlobalOptimizationGraph::addBlockGPS(int msg_index)//(const sensor_msgs::NavSatFix& GPS_msg)
{

    int enable_gps = this->fSettings["ENABLE_GPS"];
    if (enable_gps == 0) 
    {
        LOG(INFO)<<"GPS disabled in GlobalOptimizationGraph::addBlockGPS().CHECK CONFIG FILE."<<endl;
        return;
    }

    if(slam_vertex_index==0)
    {
        LOG(WARNING) <<"In addBlockGPS():slam not ready,return."<<endl;
        return;
    }

    LOG(INFO)<<"In addBlockGPS():adding gps pos factor"<<endl;
    auto GPS_msg = pGPS_Buffer->at(msg_index);
    if (!gps_msg_is_valid(GPS_msg))//unstable observation.
    {
        LOG(INFO)<<"GPS msg check failed.Return."<<endl;
        return;
    }

    if (last_gps_vertex_index< 0 && !gps_expand_ever_init)//初始化GPS.
    {
        GPS_coord.expandAt(GPS_msg.longitude, GPS_msg.latitude, GPS_msg.altitude); //这里已经初始化了altitude.
        LOG(INFO)<<"Initializing GPS expand at:"<<GPS_msg.longitude<<","<<GPS_msg.latitude<<","<<GPS_msg.altitude<<endl;
        cout <<"Initializing GPS block in Optimization Graph!"<<endl;
        gps_expand_ever_init = true;
    }

//    if(this->allow_gps_usage == false || this->gps_init_success == false||  !(this->status&0x01)  )//check if GPS valid.
//    {
//        cout<<"[WARNING] Unable to add GPS edge.GPS usage is forbidden in config file."<<endl;
//        if(this->allow_gps_usage)
//        {
//            cout<<"trying to reinit gps:"<<endl;
//            this->try_reinit_gps();
//        }
//        return;
//    }
    //新的方法.
    bool add_match_success = this->p_gps_slam_matcher->addMatch(slam_vertex_index-1, msg_index);
    LOG(INFO)<<"Add match result:"<<add_match_success<<endl;
    bool inGPSLoopMode_out = false;
    int newestState = this->p_state_tranfer_manager->updateState(add_match_success,inGPSLoopMode_out);

    if(newestState == this->p_state_tranfer_manager->STATE_NO_GPS)
    { // no valid GPS received.
        LOG(INFO)<<"Running in addBlockGPS() branch newstate == STATE_NO_GPS"<<endl;
        return;
    }

    if(newestState == this->p_state_tranfer_manager->STATE_INITIALIZING)
    {
        //add relative_pos only.
        LOG(INFO)<<"Running in addBlockGPS() branch newstate == STATE_INITIALIZING"<<endl;
        return;//TODO:暂定.未来加入这种情况的绝对位置确定.
    }

    if (newestState == this->p_state_tranfer_manager->STATE_WITH_GPS)
    {
        LOG(INFO)<<"Running in addBlockGPS() branch newstate == STATE_WITH_GPS"<<endl;
        if(inGPSLoopMode_out)
        {//GPS 环形约束形式.
            LOG(INFO)<<"In addBlockGPS():in loop mode."<<endl;
            //add pos and refine whole map.TODO.
            //step<1>.插入常规点约束.
            //double dx,dy;
            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();

            bool init_yaw_valid_ = false;//好像gps这块得乘个-1...
            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);//这是和是否回环没关系的.
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);//这是和是否回环没关系的.
            LOG(INFO)<<"in addBlockGPS():  yaw_init_to_gps value:"<<yaw_init_to_gps<< "init_yaw_valid_:"<<init_yaw_valid_<<endl;

            LOG(INFO) <<"setting gps measurement!"<<endl;
            LOG(INFO) <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            double dx,dy,dh;
            dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //     reproject to gog coordinate.
            dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            dh = gps_measurement_vec3d[2];

            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
            LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;
            if(!init_yaw_valid_)//移到这里判断,以便于留下LOG.
            {
                LOG(WARNING)<<"init yaw not valid.can not add gps measurement"<<endl;
                return;
            }
            graph.add(GPSPose2Factor(Symbol('x',slam_vertex_index-1),//gtsam::Symbol('x',slam_vertex_index),
                                                                             Point2(//gps_measurement_vec3d[0],    gps_measurement_vec3d[1]        
                                                                             dx,dy), gpsModel));
            //高度信息模型.
            noiseModel::Diagonal::shared_ptr gps_altitude_model = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[8],0.0));//第二个数没用到,随便填的
            graph.add(GPSAltitudeFactor(Symbol('h',slam_vertex_index-1),Point2(dh,0.0)//第二个数没用到,随便填的
                        ,gps_altitude_model));

            cout <<"insert pos at "<<slam_vertex_index-1<<"."<<endl;    
            cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
            cout <<"graph.size()"<<graph.size()<<endl;
            last_gps_vertex_index = slam_vertex_index-1;
            cout<<"[GPS_INFO]GPS_relative_pos:"<<
                gps_measurement_vec3d[0]<<","<<
                gps_measurement_vec3d[1]<<","<<
                delta_alt<<endl;

            LOG(INFO)<<"Loop mode GPS insertion step<1> finished."<<endl;

            //step<2>.插入gps测算的转角,位移约束,纠正这期间SLAM yaw的总漂移.
            //noiseModel::Diagonal::shared_ptr ...
            if(this->p_state_tranfer_manager->getLastGPSSLAMMatchID()<0)
            { //first time initializing GPS-SLAM matching. No loop to deal with.
                LOG(INFO)<<"Initializing GPS-SLAM matching for the 1st time.Nothing to do in addBlockGPS-Initializing-Step2."<<endl;
                return;
            }
            auto gps_covar1_ = this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).position_covariance;//at [0,4,9].
            auto gps_covar2_ = this->pGPS_Buffer->at(msg_index).position_covariance;
            int slam_node_index_loop_ = this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).slam_index;//回环点的index.
            double noise_dx_ = sqrt(gps_covar1_[0]*gps_covar1_[0] + gps_covar2_[0]*gps_covar2_[0]);
            double noise_dy_ = sqrt(gps_covar1_[4]*gps_covar1_[4] + gps_covar2_[4]*gps_covar2_[4]);
            noiseModel::Diagonal::shared_ptr noise_model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(noise_dx_,noise_dy_,0.01744*2 //2 deg /Loop
																	) );//variance for diff rotation and translation.//TODO fill in angle covariance.
            noiseModel::Diagonal::shared_ptr noise_model_relative_altitude_ = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1,0.001));//第二个数随便填的,没用到.

            //By using Diagonal::Sigmas, here we do assert that the variance of x,y and yaw are perpendicular.         
            double delta_lon_relative,delta_lat_relative,delta_alt_relative;
            //回环是旧到新.Index也应该对应.
            delta_lon_relative =  this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).longitude - this->pGPS_Buffer->at(msg_index).longitude;
            delta_lat_relative =  this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).latitude - this->pGPS_Buffer->at(msg_index).latitude;
            delta_alt_relative =  this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).altitude - this->pGPS_Buffer->at(msg_index).altitude;
            Vector3d gps_measurement_vec3d_diff_(delta_lon_relative*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat_relative*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt_relative);
            
            double diff_x,diff_y,diff_yaw;
            diff_x = gps_measurement_vec3d_diff_[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d_diff_[1]*sin(yaw_init_to_gps);
            diff_y = gps_measurement_vec3d_diff_[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d_diff_[0]*sin(yaw_init_to_gps);
            double newest_yaw_diff_rad,newest_yaw_diff_rad_variance;
            this->p_state_tranfer_manager->getNewestYawAndVarianceRad(newest_yaw_diff_rad,newest_yaw_diff_rad_variance);
            double yaw_error_fix_ = newest_yaw_diff_rad - yaw_init_to_gps;//过程中SLAM yaw产生的漂移.
            fix_angle(yaw_error_fix_);
            diff_yaw = get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_node_index_loop_)) - yaw_error_fix_;//过程中飞机头部转向的角度.SLAM的角度有漂移,要通过GPS测量纠正后再输入.
            fix_angle(diff_yaw);
            graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',slam_node_index_loop_),Symbol('x',slam_vertex_index-1),Pose2( diff_x,diff_y,diff_yaw),noise_model_relative_movement);
            graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_node_index_loop_),Symbol('h',slam_vertex_index-1),Point2(delta_alt_relative,0),noise_model_relative_altitude_);
            //graph.emplace_shared<BetweenFactor<Rot2> >(Symbol('y'),...)//TODO:纠正yaw的误差积累.
            LOG(INFO)<<"Loop mode GPS insertion step<2> finished."<<endl;
            //this->p_isam->update();Values currentEstimate = p_isam->calculateBestEstimate();

        }
        else
        {   // normal operation.
            LOG(INFO)<<"addingGPSBlock():In ordinary mode."<<endl;
            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
            bool init_yaw_valid_ = false;

            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);

            LOG(INFO)<<"in addBlockGPS():  yaw_init_to_gps value:"<<yaw_init_to_gps<< "init_yaw_valid_:"<<init_yaw_valid_<<endl;
            LOG(INFO) <<"setting gps measurement!"<<endl;
            LOG(INFO) <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            double dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //project to gog coordinate.
            double dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            double dh = gps_measurement_vec3d[2];

            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));

            {//debug only.
                auto ps__ = pSLAM_Buffer->at(slam_vertex_index-1).pose.position;
                LOG(INFO)<<"GPS_MEASUREMENT_DEBUG:dxdydh:"<<dx<<","<<dy<<","<<dh<<";"<<"SLAM:"<<ps__.x<<","<<ps__.y<<","<<ps__.z<<endl;
            }

            LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;

            //TODO potential bug here
//            if(!init_yaw_valid_)
//            {
//                LOG(WARNING)<<"init yaw not valid.can not add gps measurement"<<endl;
//                return;
//            }

            graph.add(GPSPose2Factor(Symbol('x', slam_vertex_index-1), Point2(dx, dy), gpsModel));
            noiseModel::Diagonal::shared_ptr gps_altitude_model = noiseModel::Diagonal::Sigmas(gtsam::Vector2(GPS_msg.position_covariance[8],0.0));//2nd parameter is not used, picked arbitrarily
            graph.add(GPSAltitudeFactor(Symbol('h',slam_vertex_index-1), Point2(dh,0.0), gps_altitude_model)); //GPS height relative change

            //插入优化器:(可能需要新建Edge类型)
            const double loop_variance_deg = 1; //TODO:move into config file.
            noiseModel::Diagonal::shared_ptr model_yaw_fix = noiseModel::Diagonal::Sigmas(gtsam::Vector3(GPS_msg.position_covariance[0],GPS_msg.position_covariance[4],0.01744*loop_variance_deg));
            double yaw_slam_diff = get_yaw_from_slam_msg(pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(this->p_gps_slam_matcher->at(0).slam_index));
            graph.emplace_shared<BetweenFactor<Pose2>>(Symbol('x', this->p_gps_slam_matcher->at(0).slam_index),
                                                       Symbol('x', slam_vertex_index-1),
                                                       Pose2(dx, dy, yaw_slam_diff - this->current_yaw_slam_drift),
                                                       model_yaw_fix);

            //calc YAW correction.
            bool should_update_yaw_correction = p_state_tranfer_manager->get_should_update_yaw_correction(this->slam_vertex_index-1); //检查是否建议更新yaw.
            if(should_update_yaw_correction)
            {
                int last_slam_id_out,last_match_id_out;//这两个是上一次纠正的最后id.
                this->p_state_tranfer_manager->get_last_yaw_correction_slam_id(last_slam_id_out,last_match_id_out);//从这一点开始匹配.
                //尝试进行GPS-SLAM 匹配.如果成功:更新.否则反复尝试.
                bool update_success_output= false;
                int last_match_stored_in_matcher_id = this->p_gps_slam_matcher->matchLen()-1;
                double new_deg_output,new_deg_output_variance;
                this->p_gps_slam_matcher->check2IndexAndCalcDeltaDeg(last_match_id_out,last_match_stored_in_matcher_id,//id
                                                       GPS_coord,update_success_output,new_deg_output,new_deg_output_variance
                                                         );//尝试计算yaw.
                if(update_success_output)
                {
                    double _rad = new_deg_output*180/3.1415926535;
                    double _rad_variance = new_deg_output_variance*180/3.1415926535;
                    LOG(INFO)<<"YAW_FIX_SUCCESS in match between match_id:"<<last_match_id_out<<","<<last_match_stored_in_matcher_id<<endl;
                    LOG(INFO)<<"YAW_NEWEST_VAL:"<<new_deg_output<<" deg.Variance:"<<new_deg_output_variance<<" deg."<<endl;
                    this->current_yaw_slam_drift = new_deg_output*3.1415926535/180.0 - yaw_init_to_gps; //update drift.
                    LOG(INFO)<<"Insert yaw measurement fix in GPS-local Loop.Fix Value:"<<this->current_yaw_slam_drift*180/3.1415926535<<"deg."<<endl;
                    this->p_state_tranfer_manager->set_last_slam_yaw_correction_id(this->slam_vertex_index-1,last_match_stored_in_matcher_id);
                }
                else
                {
                    LOG(INFO)<<"YAW_FIX_FAILED.ID:"<< last_match_id_out<<","<<last_match_stored_in_matcher_id<<";Values:"<<new_deg_output<<" deg.Variance:"<<new_deg_output_variance<<" deg."<<endl;
                }
            }


            cout <<"insert pos at "<<slam_vertex_index-1<<"."<<endl;
            cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
            cout <<"graph.size()"<<graph.size()<<endl;
            last_gps_vertex_index = slam_vertex_index-1;
            cout<<"[GPS_INFO]GPS_relative_pos:"<<
                gps_measurement_vec3d[0]<<","<<
                gps_measurement_vec3d[1]<<","<<
                delta_alt<<endl;
        }
    }
}

bool check_and_fix_dx(double& dx)
{
    if(dx<EPS&&dx>0)
    {    
        dx+=EPS;
    }    
    if(dx>-1*EPS && dx<0)
    {    
        dx-= EPS; 
    } 
}
double calc_angle_variance(double x,double y,double x_var,double y_var)
{
    double norm = sqrt(x*x+y*y);
    double var_norm = sqrt(x_var*x_var+y_var*y_var);
    return var_norm/(norm+EPS);
}


double get_yaw_from_slam_msg(const geometry_msgs::PoseStamped &m)
{
    auto orient = m.pose.orientation;
    Eigen::Quaterniond q_;
    q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
//    
//    Matrix3d R_SLAM_Mat = q_.toRotationMatrix();
//    Vector3d vec_forward = R_SLAM_Mat*Vector3d(1,0,0);
//    double fx,fy;//reproject to xOy;
//    fx = vec_forward[0];fy = vec_forward[1];
//    check_and_fix_dx(fx);
//    double yaw = atan2(fy,fx);
//    fix_angle(yaw);
//    return yaw;
    double roll,pitch,yaw;
    getRPYFromQuat(orient.x,orient.y,orient.z,orient.w,roll,pitch,yaw);
    return yaw;
}
void get_rpy_from_slam_msg(const geometry_msgs::PoseStamped& m,double& roll,double& pitch,double& yaw)
{
    auto orient = m.pose.orientation;
    Eigen::Quaterniond q_;
    q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
    getRPYFromQuat(orient.x,orient.y,orient.z,orient.w,roll,pitch,yaw);
}
double calcnorm(double x,double y)
{
    return sqrt(x*x+y*y);
}

const double dx_slam_var = 0.01;
const double dy_slam_var = 0.01;

const double vel_x_var = 0.05;
const double vel_y_var = 0.05;

void getDxDyFromSlamMsg(const geometry_msgs::PoseStamped& msg)
{
    
}

void GlobalOptimizationGraph::addBlockBarometer(int msg_index)
{
    auto msg = this->pBarometer_Buffer->at(msg_index);
    int __use_baro_int = this->fSettings["USE_BAROMETER"];
    bool use_baro = (__use_baro_int > 0);
    if(!use_baro)
    {
        LOG(INFO)<<"Barometer usage disabled in GOG setting."<<endl;
        return;
    }
    if(slam_vertex_index==0)
    {
       LOG(WARNING) <<"In addBlockGPS():slam not ready,return."<<endl;
       return;
    }

    //this->baro_manager.xxxx
    bool init_finished = this->p_BarometerManager->init_iterate(msg.fluid_pressure/1000);//input unit: kpa.
    //这里隐式的约定了如果有Barometer消息,则从开始就有.不存在中间发布的情况;且开始时高度变化是很小的.
    if(!init_finished)
    {
        LOG(INFO)<<"Barometer Manager still initializing."<<endl;
        return;
    }
    bool data_valid;
    double height = this->p_BarometerManager->get_current_baro_height(msg.fluid_pressure/1000,data_valid);
    if(!data_valid)
    {
        LOG(INFO)<<"Barometer info invalid!"<<endl;
        return;
    }
    if (last_baro_vertex_index<0)//初始化Baro,并且不插入值.
    {   
        //GPS_coord.expandAt(GPS_msg.longitude,GPS_msg.latitude,GPS_msg.altitude); //这里已经初始化了altitude.
        cout <<"Initiating Barometer block in Optimization Graph!"<<endl; //TODO:记录气压计和SLAM的初值差.
    }
    else
    {
        bool hist_avail;
        double diff_height = height - this->p_BarometerManager->get_current_baro_height(this->pBarometer_Buffer->at(msg_index-1).fluid_pressure/1000,hist_avail);
        if(hist_avail)
        {
            noiseModel::Diagonal::shared_ptr model_relative_height_barometer = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0)); // TODO:挪到配置文件里.现在写死barometer的方差是1.
            graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1),Symbol('h',last_baro_vertex_index),Point2(diff_height,0.0),model_relative_height_barometer);//计算barometer变化量.
        }
    }
    //TODO:考虑是否要删除其中一种约束.
    noiseModel::Diagonal::shared_ptr model_abs_height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0));//第二个数没用到,随便填的,第一个固定1.0m
    graph.add(GPSAltitudeFactor(Symbol('h',slam_vertex_index-1),Point2(height,0.0)//第二个数没用到,随便填的
                        ,model_abs_height));
    bool gps_baro_diff_ever_init = this->p_BarometerManager->get_gps_diff_ever_init();
    if(!gps_baro_diff_ever_init)//尝试初始化GPS和气压计的差值.
    //TODO:加入策略 在气压计高度产生缓漂时,通过GPS重初始化.
    {
        if(this->last_gps_vertex_index>0&&this->p_state_tranfer_manager->getCurrentState() == this->p_state_tranfer_manager->STATE_WITH_GPS)
        {
            bool init_gps_baro_diff_success;
            this->p_BarometerManager->set_gps_to_baro_height_transformation(msg.fluid_pressure/1000,this->pGPS_Buffer->at(this->pGPS_Buffer->size()-1).altitude,init_gps_baro_diff_success);
            if(!init_gps_baro_diff_success)
            {
                LOG(WARNING)<<"Init GPS_BARO relative height failed!"<<endl;
            }
        }
    }
    last_baro_vertex_index = slam_vertex_index-1;
}

void GlobalOptimizationGraph::addBlockSLAM(int msg_index)
//if use this api,only reserve the last one.
//void GlobalOptimizationGraph::addBlockSLAM(std::vector<const geometry_msgs::PoseStamped&> SLAM_msg_list)
//for multiple msgs.
{
    //two approaches：  1.create vertex when slam arrives
    //                  2.create vertex when gps arrives。
    //choose the second approach。
    LOG(INFO)<<"In addBlockSLAM(): adding slam msg at slam_vertex_index: "<<slam_vertex_index<<"."<<endl;
    LOG(INFO)<<"msg_index: "<<msg_index<<"."<<endl;
    auto SLAM_msg = pSLAM_Buffer->at(msg_index);

    //NOTE : for comparing results and debugging
    LOG(INFO)<<"SLAM msg: "<<SLAM_msg<<endl;
    mInputSlamFile << SLAM_msg.pose.position.x<<","<<SLAM_msg.pose.position.y<<","<<SLAM_msg.pose.position.z<<endl;

    //addGOGFrame(SLAM_msg.pose.position.x,SLAM_msg.pose.position.y);//create a new map 'vertexPR'
    {
        double r,p,y;
        get_rpy_from_slam_msg(SLAM_msg,r,p,y);
        fix_angle(r);
        fix_angle(p);
        fix_angle(y);
        LOG(INFO)<<"[DEBUG] Yaw in get_yaw_from_slam_msg:"<<y*180/3.1415926<<"\tpitch:"<<p*180/3.1415926<<"\troll:"<<r*180/3.1415926<<endl;
        auto p_ = SLAM_msg.pose.position;
        auto q = SLAM_msg.pose.orientation;
        LOG(INFO)<<"[DEBUG] Full info of slam input:"<<p_.x<<","<<p_.y<<","<<p_.z<<";"<<q.x<<","<<q.y<<","<<q.z<<","<<q.w<<endl;
    }

    GOG_Frame* pF = new GOG_Frame();
    cout <<"Insert "<<slam_vertex_index<<"in initialEstimate!"<<endl;
    this->p_state_tranfer_manager->updateSlam();

    if (slam_vertex_index >0)
    {
        //initial guess of abs pos and yaw
        initialEstimate.insert(Symbol('x',slam_vertex_index),
                Pose2(SLAM_msg.pose.position.x,SLAM_msg.pose.position.y,get_yaw_from_slam_msg(SLAM_msg)//this->yaw_rad_current_estimate //这里出了问题.
				)); //here we use basic_vertex_id to represent vehicle position vertex id

        initialEstimate.insert(Symbol('h',slam_vertex_index), Point2(0,0));//initial guess of height:0.//TODO.

        //calc diff for index
        auto orient = SLAM_msg.pose.orientation;
        Eigen::Quaterniond q_;
        q_.x() = orient.x;
        q_.y() = orient.y;
        q_.z() = orient.z;
        q_.w() = orient.w;
        const geometry_msgs::PoseStamped& slam_msg_old = this->pSLAM_Buffer->at(slam_vertex_index-1);

        double diff_x = SLAM_msg.pose.position.x - slam_msg_old.pose.position.x;
        double diff_y = SLAM_msg.pose.position.y - slam_msg_old.pose.position.y;
        double diff_height = SLAM_msg.pose.position.z - slam_msg_old.pose.position.z;
        double diff_yaw = (get_yaw_from_slam_msg(SLAM_msg) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)));

        double slam_xy_noise_m = this->fSettings["SLAM_RELATIVE_XY_VARIANCE_m"];
        double slam_height_noise_m = this->fSettings["SLAM_RELATIVE_HEIGHT_VARIANCE_m"];
        double slam_yaw_noise_deg = this->fSettings["SLAM_RELATIVE_YAW_VARIANCE_deg"];

        //tune parameters
        //noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
        noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(slam_xy_noise_m,slam_xy_noise_m,0.01744*slam_yaw_noise_deg));// recommended value:10mm/frame 0.01744 1.5deg/frame.
        noiseModel::Diagonal::shared_ptr model_relative_height_ = noiseModel::Diagonal::Sigmas(gtsam::Vector2(slam_height_noise_m,0));//1mm/frame
        LOG(INFO)<<"SLAM relative noise setting xy(m):"<<slam_xy_noise_m<<",yaw(deg):"<<slam_yaw_noise_deg<<",height noise(m):"<<slam_height_noise_m<<endl;

        //'x': xOy, horizontal position;
        //'h': height;
        //'y': yaw offset.
        graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x', slam_vertex_index-1), Symbol('x',slam_vertex_index), Pose2(diff_x, diff_y, diff_yaw), model_relative_movement);
        graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1), Symbol('h',slam_vertex_index), Point2(diff_height, 0.0), model_relative_height_);//the second parameter is picked arbitraly

        //graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x', slam_vertex_index), Symbol('x',slam_vertex_index-1), Pose2( diff_x, diff_y, diff_yaw), model_relative_movement);
        //graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index), Symbol('h',slam_vertex_index-1), Point2(diff_height, 0.0), model_relative_height_);//the second parameter is picked arbitraly

        {//仅供实验:在没有GPS约束的时候,给一个很弱的绝对位置约束.
            int newestState = this->p_state_tranfer_manager->getCurrentState();
            if(newestState != this->p_state_tranfer_manager->STATE_WITH_GPS)
            {
                LOG(WARNING)<<"In addBlockSLAM():GPS not stable;adding slam prior pose restriction."<<endl;
                auto p__ = SLAM_msg.pose.position;
                //noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1,0.1,0.01744*10)); //0.1m,10度.
                noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(1, 1, 1)); //0.1m,10度.
                graph.emplace_shared<PriorFactor<Pose2> >(Symbol('x',slam_vertex_index),Pose2(p__.x,p__.y,get_yaw_from_slam_msg(SLAM_msg)),priorNoise_Absolute);//位置和朝向角都初始化成0.
                noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0)); //高度方差:1.
                graph.emplace_shared<PriorFactor<Point2> >(Symbol('h',slam_vertex_index),Point2(p__.z,0),priorNoise_Height);//高度初始化成0
            }
        }

        //graph.emplace_shared<BetweenFactor<Rot2> >(Symbol('y',slam_vertex_index-1),Symbol('y',slam_vertex_index),....);// offset in heading
        slam_vertex_index++;
        if(online_mode)
        {
            p_isam->update(graph, initialEstimate);
    
            // FEJ
            // ISAM2Params params_;
            // parameters.relinearizeThreshold = 0.01;
            // parameters.relinearizeSkip = 1;

            Values currentEstimate = p_isam->calculateEstimate();//p_isam->estimate();
            int current_dof = 0;
            double current_chi2 = chi2_red(graph,currentEstimate, current_dof);
            LOG(INFO)<<"[Optimizer INFO] Current dof and chi2:"<<current_dof<<","<<current_chi2<<endl;
            LOG(INFO)<<"current yaw_init_to_slam:"<<yaw_init_to_slam*180/3.14159<<" deg."<<endl;
            cout <<"last state:"<<endl;
            const Pose2* p_obj = &(currentEstimate.at(Symbol('x',slam_vertex_index-1)).cast<Pose2>());
                           //dynamic_cast<Pose2*>( &(currentEstimate.at(slam_vertex_index-1)) );//这几个值都不对,应该是内存错误.
                           //(Pose2*) (&p_isam->calculateEstimate(slam_vertex_index-1));
                           //dynamic_cast<Pose2> (currentEstimate.at(slam_vertex_index-1));
            LOG(INFO)<<"Current NODE ESTIMATED STATE at index:"<<slam_vertex_index-1<< " x:"<<p_obj->x()<<",y:"<<p_obj->y()<<",theta:"<<p_obj->theta()<<endl;
            LOG(INFO)<<"Current NODE ESTIMATED Position for visualizer:"<<p_obj->x()<<","<<p_obj->y()<<","<<p_obj->theta()*10<<endl;

            //currentEstimate.print("Current estimate: ");

            LOG(INFO)<<"Online Mode!"<<endl;

            // fetch values from optimizer
            {
                state_mutex.lock();
                LOG(WARNING)<<"Changing current_status output."<<endl;
                //    bool state_correct = false;
                //    Quaterniond ret_val_R;
                //    Vector3d ret_val_t;
                //    std_msgs::Header header_;
                //    int innerID_of_GOG;
                this->current_status.state_correct = true;//TODO.
                auto Q__ = SLAM_msg.pose.orientation;

                //only affect the output message;will not change anything in optimization graph itself.
                const Pose2 current_pose2d = currentEstimate.at(Symbol('x',slam_vertex_index-1)).cast<Pose2>();
                double new_yaw_rad = current_pose2d.theta();
                this->yaw_rad_current_estimate = new_yaw_rad;
                double newx, newy, newz, neww;
                //LOG(WARNING)<<"CHANGING newxyzw for DEBUG QUAT ONLY!!!"<<endl;
                //newx = Q__.x;newy=Q__.y;newz=Q__.z;neww=Q__.w;//Debug.
                //尝试如果不改变xyzw,是否坐标系仍然不正常.
                getNewQuaternionFromOriginalQuaternionAndNewYawAngle(Q__.x,Q__.y,Q__.z,Q__.w,new_yaw_rad,
                                                             newx,newy,newz,neww);
                
                current_status.ret_val_R.x() = newx;
                current_status.ret_val_R.y() = newy;
                current_status.ret_val_R.z() = newz;
                current_status.ret_val_R.w() = neww;
    
                current_status.ret_val_t[0] = current_pose2d.x();
                current_status.ret_val_t[1] = current_pose2d.y();
                Point2 current_height = currentEstimate.at(Symbol('h',slam_vertex_index-1)).cast<Point2>();
                current_status.ret_val_t[2] = current_height.x();//y没用.
                current_status.header_ = SLAM_msg.header;
                current_status.innerID_of_GOG = this->slam_vertex_index-1;

                //TODO:dump current_status to a log file.
                LOG(INFO)<<"Current_status output changed!"<<endl;

                state_mutex.unlock();
            }


            /*if(slam_vertex_index%1000 == 0)
            {
                GaussNewtonParams parameters;
                // Stop iterating once the change in error between steps is less than this value
                parameters.relativeErrorTol = 1e-5;
                // Do not perform more than N iteration steps
                parameters.maxIterations = 10000;
                // Create the optimizer ...
                GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
                Values result= optimizer.optimize();
            }*/
    
//            if(slam_vertex_index%300 == 0)
//            {
//                stringstream ss;
//                ss<<"Pose2SLAMExample_"<<slam_vertex_index/300<<".dot";
//                string path;
//                ss>>path;
//                ofstream os(path.c_str());
//                //graph.bayesTree().saveGraph(os, currentEstimate);
//                p_isam->saveGraph(path.c_str());
//                //导出g2o图文件.
//                stringstream ss_g2o;
//                ss_g2o<<"Pose2SLAMExample_"<<slam_vertex_index/300<<".g2o";
//                string g2o_path;
//                ss_g2o>>g2o_path;
//                writeG2o(graph,currentEstimate,g2o_path.c_str());
//            }
            graph.resize(0);
            initialEstimate.clear();
            cout<<"-------- ---------------------------------- --------"<<endl;
        }
        else
        {
            const int optimize_count = 1800;
            if(slam_vertex_index==optimize_count)
            {
                LOG(INFO)<<"In offline mode:index == "<<optimize_count<<".start optimization."<<endl;
                GaussNewtonParams parameters;
                // Stop iterating once the change in error between steps is less than this value
                parameters.relativeErrorTol = 1e-5;
                // Do not perform more than N iteration steps
                parameters.maxIterations = 1000;
                // Create the optimizer ...
                //p_offline_optimizer = new GaussNewtonOptimizer(graph, initialEstimate, parameters);
                GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
                Values offline_result = optimizer.optimize();
                for(int i = 0;i<optimize_count;i++)
                {
                    Pose2 var_x = offline_result.at(Symbol('x',i)).cast<Pose2>();
                    Point2 var_h = offline_result.at(Symbol('h',i)).cast<Point2>();
                    LOG(INFO)<<"    [OFFLINE OPTIMIZED RESULT] node_id:"<<i<<";xyz:"<<var_x.x()<<","<<var_x.y()<<","<<var_h.x()<<";yaw:"<<var_x.theta()*180/3.1415926535<<" deg."<<endl;
                }
//                offline_result.print("Final Result:\n");
//                stringstream ss;
//                ss<<"Offline_"<<optimize_count<<".dot";
//                string path;
//                ss>>path;
//                ofstream os(path.c_str());
//                graph.saveGraph(os);
//                //导出g2o图文件.
//                stringstream ss_g2o;
//                ss_g2o<<"Offline_"<<optimize_count<<".g2o";
//                string g2o_path;
//                ss_g2o>>g2o_path;
//                writeG2o(graph,offline_result,g2o_path.c_str());
            }
        }
    }
    else
    //SLAM输入初始化。
    //设置初始朝向角为0度，位置为0,0.以后的位姿以此为参考。
    {
        initialEstimate.insert(Symbol('x',slam_vertex_index),Pose2(//initial guess of abs pos and yaw
                                                   SLAM_msg.pose.position.x,SLAM_msg.pose.position.y,0 
				)); //here we use basic_vertex_id to represent vehicle position vertex id
        initialEstimate.insert(Symbol('h',slam_vertex_index),Point2(0,0));//initial guess of height:0.//TODO.
        LOG(INFO)<<"Initializing slam factor."<<endl;
        //noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3,0.3,0.01744*5));//初始化角度方差5度. 0.3m.
        noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.00001,0.00001,0.01744*0.00001)); //0.1m,10度.
        graph.emplace_shared<PriorFactor<Pose2> >(Symbol('x',0),Pose2(0,0,0),priorNoise_Absolute);//位置和朝向角都初始化成0.
        //noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0,0.0));
        noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.00001,0.0));
        graph.emplace_shared<PriorFactor<Point2> >(Symbol('h',0),Point2(0,0),priorNoise_Height);//高度初始化成0

        noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1,0);
        Rot2 rot_drift_prior = Rot2::fromAngle(0);
        //initialEstimate.insert(Symbol('y',0),...)//TODO.
        //graph.emplace_shared<PriorFactor<Rot2> >(Symbol('y',0),rot_drift_prior,priorNoise_Height);//定义初始SLAM yaw漂移角为0.
        double current_yaw_slam = get_yaw_from_slam_msg(SLAM_msg);

        slam_vertex_index++;
    }
    //if gps-like measurement inputs:  (pay attention:here we use Point2 as input.)
    //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
    //graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel)); //add observation
}

void GlobalOptimizationGraph::addBlockQRCode()
{
    //pEdgeQRCode = 
}
void addBlockSceneRetriever_StrongCoupling(); //Solve se(3) from multiple points PRXYZ;
void addBlockSceneRetriever_WeakCoupling();//just do square dist calc.

//void GlobalOptimizationGraph::addBlockSceneRetriever()
void GlobalOptimizationGraph::addBlockLoop(const LoopMessage& msg)
{/*
    Quaterniond quat_;
    quat_.x() = msg.x;quat_.y() = msg.y;quat_.z() = msg.z;quat_.w() = msg.w;
    //here we solve yaw first, then calc yaw diff again.
    //caution:no yaw info inside loop_msg,so maybe we should change the logic of ros_global_optimization.cpp in scene_retriever.

    //
    auto Mat1 = this->pSLAM_Buffer->at(msg.prev_gog_frame_id).orientation.toRotationMatrix....;//todo:syntax....
    auto Mat2 = Mat1* quat_.toRotationMatrix();
    double yaw2 = get_yaw_from_Mat(Mat2);//estimation of fixed yaw.//todo:get_yaw_from_Mat...

    //TODO:we could check pitch and roll of Mat2, and judge if this loop is correct.if pitch and roll of mat2 is too far from get_rpy_from_slam_msg(this->pSLAM_Buffer->at(msg.loop_gog_frame_id)), this loop may be a false-positive.
    double yaw1 = get_yaw_from_slam_msg(this->pSLAM_Buffer->at(msg.prev_gog_frame_id));

    double diff_yaw = yaw2 - yaw1;    
    graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',msg.prev_gog_frame_id),Symbol('x',msg.loop_gog_frame_id),Pose2(msg.x,msg.y,diff_yaw),noise_model_loop_movement_xy);
    graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',msg.prev_gog_frame_id),Symbol('h',msg.loop_gog_frame_id),Point2(msg.z,0),noise_model_loop_movement_height);
    LOG(INFO)<<"Added Block Loop in optimization graph."<<endl;*/
}
bool GlobalOptimizationGraph::tryInitVelocity()
{
    cout<<"Nothing to do now!!!"<<endl;
    return false;//TODO :fill in this.
}
bool GlobalOptimizationGraph::checkAHRSValid()
{
    cout<<"Nothing to do now!!!"<<endl;
    return false;//TODO: fill in this.
}

//----------------------
void GlobalOptimizationGraph::DebugPrintWholeGraphValue()
{
        //Values result = ()//DoglegOptimizer(graph, initialEstimate).optimize();
        Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
        //result.print("Final results:\n");
        for(Values::iterator key_value = result.begin(); key_value != result.end(); ++key_value)
        {
            Key key = key_value->key;
            Symbol asSymbol(key);
            cout << asSymbol.chr() << std::endl;
            cout << asSymbol.index() << std::endl;
            if(asSymbol.chr() == 'x') // xOy position.
            {
                Pose2 p2 = key_value->value.cast<Pose2>();
                p2.print("xOy pt : ");
            } else if(asSymbol.chr() == 'h') //height.
            {
                Point2 h_ = key_value->value.cast<Point2>();
                h_.print("height vec : ");
            }
        }
    //if a 'vertex' inputs: 
    //Values initialEstimate; initialEstimate.insert(1,Pose2( abs pos and yaw));
    //if a 'relative pos' inputs:
    //graph.emplace_shared<BetweenFactor<Pose2> >(1,2,Pose2( diff_x,diff_y,diff_yaw),model_relative_movement);
    //if gps-like measurement inputs:  (pay attention:here we use Point2 as input.)
    //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
    //graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel));

    //when doing optimization:
    //  GaussNewtonParams parameters;
    //  GaussNewtonOptimizer optimizer(graph, initials, parameters);
    //  Values results = optimizer.optimize(); //get result.
    //  visualize loss function:(like 'chi2()')
    //  cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;

}

bool GlobalOptimizationGraph::doOptimization()
{
        //bool retval = false;//if optimize success(edge >=3).
        //GaussNewtonParams parameters;
        //parameters.relativeErrorTol = 1e-5;// Stop iterating once the change in error between steps is less than this value
        //parameters.maxIterations = 10000;// Do not perform more than N iteration steps
        //GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);// Create the optimizer ...
        //Values result = optimizer.optimize();// ... and optimize
        LOG(WARNING)<<"Nothing happens in GlobalOptimizationGraph::doOptimization()!"<<endl;
        return true;
}
void GlobalOptimizationGraph::addBlockVelocity(int msg_index)//(const geometry_msgs::TwistStamped& velocity_msg)
{

/*
    LOG(INFO)<<"In addBlockVelocity():adding gps vel factor"<<endl;
    auto velocity_msg = this->pVelocity_Buffer->at(msg_index);
    if(slam_vertex_index == 0)
    {
        LOG(INFO)<<"slam not ready."<<endl;
        return;
    }
    if(last_gps_vel_index == -1)
    {
        last_gps_vel_index = slam_vertex_index-1;
        LOG(INFO) << "Initialized slam-vel msg at slam msg index"<<slam_vertex_index-1<<"!"<<endl;
    }
    else
    {
        //计算差.
        double vx = velocity_msg.twist.linear.x;
        double vy = velocity_msg.twist.linear.y;
        double dt = pSLAM_Buffer->at(slam_vertex_index-1).header.stamp.toSec()- pSLAM_Buffer->at(last_gps_vel_index).header.stamp.toSec();
        double dx_vel = vx*dt;
        double dy_vel = vy*dt;
        double dx_slam = this->pSLAM_Buffer->at(slam_vertex_index-1).pose.position.x - this->pSLAM_Buffer->at(last_gps_vel_index).pose.position.x;
        double dy_slam = this->pSLAM_Buffer->at(slam_vertex_index-1).pose.position.y - this->pSLAM_Buffer->at(last_gps_vel_index).pose.position.y;
        //计算yaw.两种方法：几何法，最小二乘重投影。
        check_and_fix_dx(dx_slam); //避免出现极小。
        check_and_fix_dx(dx_vel);
        double theta_slam = atan2(dy_slam,dx_slam);
        double theta_gps = atan2(dy_vel,dx_vel);
        double theta_slam_var = calc_angle_variance(dx_slam,dy_slam,dx_slam_var,dy_slam_var);//dx_slam_var,dy_slam_var填入slam pos差分的标准差.
        double theta_gps_var = calc_angle_variance(dx_vel,dy_vel,vel_x_var,vel_y_var);

        //先计算theta_slam_to_gps;再变换到theta_init_to_gps;
        LOG(INFO)<<"In addBlockVelocity() we do update yaw: theta_gps:"<<theta_gps*180/3.14159<<"deg;theta_slam:"<<theta_slam*180/3.14159<<"deg."<<endl;
        LOG(INFO)<<"dx_vel = "<<dx_vel<<",dy_vel = "<<dy_vel<<";dx_slam = "<<dx_slam<<",dy_slam = "<<dy_slam<<endl<<endl;
        LOG(INFO)<<"original vx: "<<vx<<",vy: "<<vy<<";"<<"dt: "<<dt<<"sec."<<"current index:"<<slam_vertex_index-1<<"prev_index:"<<last_gps_vel_index<<endl;
        double theta_slam_to_gps = theta_gps - theta_slam;
        double yaw_init_to_gps = yaw_init_to_slam - theta_slam_to_gps;
        fix_angle(yaw_init_to_gps);

        LOG(INFO) << "theta slam_to_gps:"<<theta_slam_to_gps<<";yaw init to gps:"<<yaw_init_to_gps*180/3.14159<<" deg."<<endl;
        //sin(yaw_init_to_gps) cos(yaw_init_to_gps)
        double diff_x = cos(-1*yaw_init_to_gps)*dx_vel - sin(-1*yaw_init_to_gps)*dy_vel;
        double diff_y = sin(-1*yaw_init_to_gps)*dy_vel + cos(-1*yaw_init_to_gps)*dx_vel;
        LOG(INFO) << "dx_slam:"<<dx_slam<<",dy_slam:"<<dy_slam<<",dx_vel:"<<dx_vel<<",dy_vel:"<<dy_vel<<"."<<endl;
        if(calcnorm(dx_slam,dy_slam) > 0.1 && calcnorm(dx_vel,dy_vel)>0.1) //认为已经观测到有效的yaw.
        {
            LOG(INFO) <<"speed matching valid! slam_vertex_index-1: "<<slam_vertex_index-1<<endl;
            //diff_yaw = yaw_init_to_gps - last_yaw_init_to_gps; // last_yaw_init_to_gps //这个应该不产生yaw的观测，因为机身的转动不可观测。
            cout <<"insert vel at "<<slam_vertex_index-1<<"."<<endl;
            cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
            noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
            graph.emplace_shared<BetweenFactor<Pose2> >(last_gps_vel_index,slam_vertex_index-1,Pose2(diff_x,diff_y,theta_slam_to_gps - yaw_init_to_gps),model_relative_movement);
        }
        last_gps_vel_index = slam_vertex_index-1;
    }
*/
}


#endif
