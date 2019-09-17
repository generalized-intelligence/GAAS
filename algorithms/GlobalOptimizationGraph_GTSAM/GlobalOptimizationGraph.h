#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H
#define EIGEN_MAX_STATIC_ALIGN_BYTES 0  //To avoid Eigen memory alignment error.

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
//#include <gtsam/geometry/Rot2.h>
//#include <gtsam/geometry/Pose2.h>
//#include <gtsam/geometry/Point2.h>


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

using namespace std;
using namespace ygz;
using namespace gtsam;
const float thHuber = 50;

const int GPS_INIT_BUFFER_SIZE = 100;
using namespace ygz;


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
    void addBlockSceneRetriever();
    void addBlockFCAttitude();
    
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
    //status management.
public:
    static const int STATUS_NO_GPS_NO_SCENE = 0; // a bit map.
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
    const int relinearizeInterval = 2000;//300;
    //NonlinearISAM* p_isam;//(relinearizeInterval);
    //NonlinearISAM* p_isam;
    ISAM2* p_isam;

    int status = STATUS_NO_GPS_NO_SCENE;

    int slam_vertex_index = 0;
    int last_gps_vertex_index = -1;
    int last_gps_vel_index = -1;
    int last_baro_vertex_index = -1;
    double yaw_init_to_slam = 0.0; //GOG初始的y轴 转向 slam的y轴夹角.
    //double yaw_init_to_gps = 0.0; //GOG的y轴 到 gps的北 夹角.
    
    //message buffers.
    
    CallbackBufferBlock<geometry_msgs::PoseStamped> * pSLAM_Buffer;
    CallbackBufferBlock<sensor_msgs::NavSatFix> * pGPS_Buffer;
    CallbackBufferBlock<geometry_msgs::TwistStamped> * pVelocity_Buffer;
    CallbackBufferBlock<nav_msgs::Odometry> * pAHRS_Buffer;
    CallbackBufferBlock<sensor_msgs::FluidPressure>* pBarometer_Buffer;
    //uav location attitude info management.
    void remap_UAV_coordinate_with_GPS_coordinate();
    GPSExpand GPS_coord;

    bool allow_gps_usage = true;//can be configured by config.yaml
    double gps_init_longitude,gps_init_latitude,gps_init_altitude;

    double gps_init_lon_variance,gps_init_lat_variance,gps_init_alt_variance;
    std::vector<sensor_msgs::NavSatFix> gps_info_buffer;
    //time for calc speed vertex
    double last_slam_msg_time;
    double init_slam_msg_time;

    OptimizationGraphStatusT current_status;
    std::mutex state_mutex;//锁定当前返回值的状态.
};

GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{
    //cv::FileStorage fSettings;//(string(argv[1]),cv::FileStorage::READ);
    this->fSettings.open(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];

    //p_isam = new NonlinearISAM (relinearizeInterval);
    ISAM2Params isam2_params_; //在定義ISAM2例項的時候儲存引數的。
    int enable_relinearize_in = this->fSettings["ENABLE_RELINEARIZE"];
    bool enable_relinearize = (enable_relinearize_in!=0);
    double relinearizeThres = this->fSettings["RELINEARIZE_THRES"];
    isam2_params_.relinearizeThreshold = relinearizeThres; //0.01;
    int relinearizeSkip_num = this->fSettings["RELINEARIZE_SKIP_NUM"];
    isam2_params_.relinearizeSkip = relinearizeSkip_num;//1;//多少个变量之后,relinearize.
    isam2_params_.enableRelinearization = enable_relinearize;//false;//禁止relinearize.

    p_isam = new ISAM2(isam2_params_);
    
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
    LOG(INFO)<<"In addBlockGPS():adding gps pos factor"<<endl;
    auto GPS_msg = pGPS_Buffer->at(msg_index);
    if (!gps_msg_is_valid(GPS_msg))//unstable observation.
    {
        LOG(INFO)<<"GPS msg check failed.Return."<<endl;
        return;
    }
    if(slam_vertex_index==0)
    {
       LOG(WARNING) <<"In addBlockGPS():slam not ready,return."<<endl;
       return;
    }
    if (last_gps_vertex_index<0)//初始化GPS.
    {
        GPS_coord.expandAt(GPS_msg.longitude,GPS_msg.latitude,GPS_msg.altitude); //这里已经初始化了altitude.
        cout <<"Initiating GPS block in Optimization Graph!"<<endl;
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
    bool add_match_success = this->p_gps_slam_matcher->addMatch(slam_vertex_index-1,msg_index);
    LOG(INFO)<<"Add match result:"<<add_match_success<<endl;
    bool inGPSLoopMode_out = false;
    int newestState = this->p_state_tranfer_manager->updateState(add_match_success,inGPSLoopMode_out);
    if(newestState == this->p_state_tranfer_manager->STATE_NO_GPS)
    { // 无有效GPS信号.
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

            LOG(INFO) <<"setting gps measurement!"<<endl;
            LOG(INFO) <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            double dx,dy,dh;
            dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) + gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //     reproject to gog coordinate.
            dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) - gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            dh = gps_measurement_vec3d[2];

            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
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
            noiseModel::Diagonal::shared_ptr gps_altitude_model = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[8],0.0));//第二个数没用到,随便填的
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
            noiseModel::Diagonal::shared_ptr noise_model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(noise_dx_,noise_dy_,0.1) );//variance for diff rotation and translation.//TODO fill in angle covariance.
            noiseModel::Diagonal::shared_ptr noise_model_relative_altitude_ = noiseModel::Diagonal::Sigmas(Vector2(0.1,0.001));//第二个数随便填的,没用到.

            //By using Diagonal::Sigmas, here we do assert that the variance of x,y and yaw are perpendicular.         
            double delta_lon_relative,delta_lat_relative,delta_alt_relative;
            delta_lon_relative =  this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).longitude - this->pGPS_Buffer->at(msg_index).longitude;
            delta_lat_relative =  this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).latitude - this->pGPS_Buffer->at(msg_index).latitude;
            delta_alt_relative =  this->pGPS_Buffer->at( this->p_gps_slam_matcher->at(p_state_tranfer_manager->getLastGPSSLAMMatchID()).gps_index ).altitude - this->pGPS_Buffer->at(msg_index).altitude;
            Vector3d gps_measurement_vec3d_diff_(delta_lon_relative*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat_relative*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt_relative);
            
            double diff_x,diff_y,diff_yaw;
            diff_x = gps_measurement_vec3d_diff_[0]*cos(yaw_init_to_gps) + gps_measurement_vec3d_diff_[1]*sin(yaw_init_to_gps);
            diff_y = gps_measurement_vec3d_diff_[1]*cos(yaw_init_to_gps) - gps_measurement_vec3d_diff_[0]*sin(yaw_init_to_gps);
            double newest_yaw_diff_rad,newest_yaw_diff_rad_variance;
            this->p_state_tranfer_manager->getNewestYawAndVarianceRad(newest_yaw_diff_rad,newest_yaw_diff_rad_variance);
            double yaw_error_fix_ = newest_yaw_diff_rad - yaw_init_to_gps;//过程中SLAM yaw产生的漂移.
            fix_angle(yaw_error_fix_);
            diff_yaw = get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_node_index_loop_)) - yaw_error_fix_;//过程中飞机头部转向的角度.SLAM的角度有漂移,要通过GPS测量纠正后再输入.
            fix_angle(diff_yaw);
            graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',slam_vertex_index-1),Symbol('x',slam_node_index_loop_),Pose2( diff_x,diff_y,diff_yaw),noise_model_relative_movement);
            graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1),Symbol('h',slam_node_index_loop_),Point2(delta_alt_relative,0),noise_model_relative_altitude_);
            //graph.emplace_shared<BetweenFactor<Rot2> >(Symbol('y'),...)//TODO:纠正yaw的误差积累.
            LOG(INFO)<<"Loop mode GPS insertion step<2> finished."<<endl;
            //this->p_isam->update();Values currentEstimate = p_isam->calculateBestEstimate();

        }//GPS 环形约束形式结束.
        else
        {//常规操作.
            LOG(INFO)<<"addingGPSBlock():In ordinary mode."<<endl;
            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
            bool init_yaw_valid_ = false;
            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);
            //好像也是符号问题.
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);
            LOG(INFO) <<"setting gps measurement!"<<endl;
            LOG(INFO) <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            double dx,dy,dh;
            dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) + gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //     reproject to gog coordinate.
            dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) - gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            dh = gps_measurement_vec3d[2];
            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
            {//debug only.
                auto ps__ = pSLAM_Buffer->at(slam_vertex_index-1).pose.position;
                LOG(INFO)<<"GPS_MEASUREMENT_DEBUG:dxdydh:"<<dx<<","<<dy<<","<<dy<<";"<<"SLAM:"<<ps__.x<<","<<ps__.y<<","<<ps__.z<<endl;
            }
            LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;
            if(!init_yaw_valid_)//移到这里判断,方便产生LOG.
            {
                LOG(WARNING)<<"init yaw not valid.can not add gps measurement"<<endl;
                return;
            }
            graph.add(GPSPose2Factor(Symbol('x',slam_vertex_index-1),//gtsam::Symbol('x',slam_vertex_index),
                                                                             Point2(//gps_measurement_vec3d[0],    gps_measurement_vec3d[1]
                                                                             dx,dy), gpsModel));
            noiseModel::Diagonal::shared_ptr gps_altitude_model = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[8],0.0));//第二个数没用到,随便填的
            graph.add(GPSAltitudeFactor(Symbol('h',slam_vertex_index-1),Point2(dh,0.0)//第二个数没用到,随便填的
                 ,gps_altitude_model));//GPS高程变化量.
            //graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1),Symbol('h',slam_vertex_index),....);//计算GPS高程变化量.

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
            noiseModel::Diagonal::shared_ptr model_relative_height_barometer = noiseModel::Diagonal::Sigmas(Vector2(1.0,0.0)); // TODO:挪到配置文件里.现在写死barometer的方差是1.
            graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1),Symbol('h',last_baro_vertex_index),Point2(diff_height,0.0),model_relative_height_barometer);//计算barometer变化量.
        }
    }
    //TODO:考虑是否要删除其中一种约束.
    noiseModel::Diagonal::shared_ptr model_abs_height = noiseModel::Diagonal::Sigmas(Vector2(1.0,0.0));//第二个数没用到,随便填的,第一个固定1.0m
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

void GlobalOptimizationGraph::addBlockSLAM(int msg_index)//(const geometry_msgs::PoseStamped& SLAM_msg)
//if use this api,only reserve the last one.
//void GlobalOptimizationGraph::addBlockSLAM(std::vector<const geometry_msgs::PoseStamped&> SLAM_msg_list)
//for multiple msgs.
{
    //两种策略：1.来slam消息就立即创建节点
    //          2.等到gps信息到来再创建。
    //这里我们选择第2种。
    LOG(INFO)<<"In addBlockSLAM(): adding slam msg at slam_vertex_index: "<<slam_vertex_index<<"."<<endl;
    auto SLAM_msg = pSLAM_Buffer->at(msg_index);
    //addGOGFrame(SLAM_msg.pose.position.x,SLAM_msg.pose.position.y);//create a new map 'vertexPR'
    double current_yaw_slam = get_yaw_from_slam_msg(SLAM_msg);
    if(current_yaw_slam<0)
    {
        current_yaw_slam+=(3.1415926*2);
    }
    LOG(INFO)<<"[DEBUG] Yaw in get_yaw_from_slam_msg:"<<current_yaw_slam*180/3.1415926<<endl;

    GOG_Frame* pF = new GOG_Frame();
    cout <<"Insert "<<slam_vertex_index<<"in initialEstimate!"<<endl;
    this->p_state_tranfer_manager->updateSlam();
    initialEstimate.insert(Symbol('x',slam_vertex_index),Pose2(//initial guess of abs pos and yaw
                                                   SLAM_msg.pose.position.x,SLAM_msg.pose.position.y,0
				)); //here we use basic_vertex_id to represent vehicle position vertex id
    initialEstimate.insert(Symbol('h',slam_vertex_index),Point2(0,0));//initial guess of height:0.//TODO.

    if (slam_vertex_index >0)
    {
        //calc diff for index
        auto orient = SLAM_msg.pose.orientation;
        Eigen::Quaterniond q_;
        q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
        Matrix3d R_SLAM_Mat = q_.toRotationMatrix();
        double fx,fy;//reproject to xOy;
        Vector3d vec_forward = R_SLAM_Mat*Vector3d(1,0,0);
        fx = vec_forward[0];fy = vec_forward[1];
        double diff_x,diff_y;
        diff_x = fx*cos(yaw_init_to_slam) - fy*sin(yaw_init_to_slam);
        diff_y = fy*cos(yaw_init_to_slam) + fx*sin(yaw_init_to_slam);
        //for index-1
        const geometry_msgs::PoseStamped& slam_msg_old = this->pSLAM_Buffer->at(slam_vertex_index-1);
        orient = slam_msg_old.pose.orientation;
        q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
        R_SLAM_Mat = q_.toRotationMatrix();
        vec_forward = R_SLAM_Mat*Vector3d(1,0,0);
        fx = vec_forward[0];fy = vec_forward[1];
        double oldx,oldy;
        oldx = fx*cos(yaw_init_to_slam) - fy*sin(yaw_init_to_slam);
        oldy = fy*cos(yaw_init_to_slam) + fx*sin(yaw_init_to_slam);
        //minus
        diff_x -= oldx;
        diff_y -= oldy;
        double diff_height = SLAM_msg.pose.position.z - slam_msg_old.pose.position.z;
        double diff_yaw = (get_yaw_from_slam_msg(SLAM_msg) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)));
        //调调参数.
        //noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
        noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.001,0.001,0.0016));// 1mm/帧 0.1°/帧
        noiseModel::Diagonal::shared_ptr model_relative_height_ = noiseModel::Diagonal::Sigmas(Vector2(0.001,0));//1mm/帧,第二个没用.
        //'x':xOy平面位置;'h':高度;'y':朝向角偏差.
        graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x', slam_vertex_index-1),Symbol('x',slam_vertex_index),Pose2( diff_x,diff_y,diff_yaw),model_relative_movement);
        graph.emplace_shared<BetweenFactor<Point2> >(Symbol('h',slam_vertex_index-1),Symbol('h',slam_vertex_index),Point2(diff_height,0.0),model_relative_height_);//高度,第二项随便填的
        //graph.emplace_shared<BetweenFactor<Rot2> >(Symbol('y',slam_vertex_index-1),Symbol('y',slam_vertex_index),....);//朝向角的偏移量.
        slam_vertex_index++;
        p_isam->update(graph,initialEstimate);

        //FEJ实现:设置parameters.relinearizeSkip
        //ISAM2Params params_; //在定義ISAM2例項的時候儲存引數的。
	//  parameters.relinearizeThreshold = 0.01;
  	// parameters.relinearizeSkip = 1;

        Values currentEstimate = p_isam->calculateEstimate();//p_isam->estimate();
        LOG(INFO)<<"current yaw_init_to_slam:"<<yaw_init_to_slam*180/3.14159<<" deg."<<endl;
        cout <<"last state:"<<endl;
        const Pose2* p_obj = &(currentEstimate.at(Symbol('x',slam_vertex_index-1)).cast<Pose2>());
                       //dynamic_cast<Pose2*>( &(currentEstimate.at(slam_vertex_index-1)) );//这几个值都不对,应该是内存错误.
                       //(Pose2*) (&p_isam->calculateEstimate(slam_vertex_index-1));
                       //dynamic_cast<Pose2> (currentEstimate.at(slam_vertex_index-1));
        LOG(INFO)<<"Current NODE ESTIMATED STATE at index:"<<slam_vertex_index-1<< " x:"<<p_obj->x()<<",y:"<<p_obj->y()<<",theta:"<<p_obj->theta()<<endl;
        LOG(INFO)<<"Current NODE ESTIMATED Position for visualizer:"<<p_obj->x()<<","<<p_obj->y()<<","<<p_obj->theta()*10<<endl;
        currentEstimate.print("Current estimate: ");
        //取出优化器里的量.
        {//在新的作用域里搞,不动外面的东西.将来可以挪到新函数里去.
            state_mutex.lock();
            LOG(INFO)<<"Changing current_status output."<<endl;
            //    bool state_correct = false;
            //    Quaterniond ret_val_R;
            //    Vector3d ret_val_t;
            //    std_msgs::Header header_;
            //    int innerID_of_GOG;
            this->current_status.state_correct = true;//TODO.
            auto Q__ = SLAM_msg.pose.orientation;
            const Pose2 current_pose2d = currentEstimate.at(Symbol('x',slam_vertex_index-1)).cast<Pose2>();
            double new_yaw_rad = current_pose2d.theta();
            double newx,newy,newz,neww;
            getNewQuaternionFromOriginalQuaternionAndNewYawAngle(Q__.x,Q__.y,Q__.z,Q__.w,new_yaw_rad,
                                                         newx,newy,newz,neww);
            
            current_status.ret_val_R.x() = newx;//生成一个.
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

        if(slam_vertex_index%300 == 0)
        {
            stringstream ss;
            ss<<"Pose2SLAMExample_"<<slam_vertex_index/300<<".dot";
            string path;
            ss>>path;
            ofstream os(path.c_str());
            //graph.bayesTree().saveGraph(os, currentEstimate);
            p_isam->saveGraph(path.c_str());
            //导出g2o图文件.
            stringstream ss_g2o;
            ss_g2o<<"Pose2SLAMExample_"<<slam_vertex_index/300<<".g2o";
            string g2o_path;
            ss_g2o>>g2o_path;
            writeG2o(graph,currentEstimate,g2o_path.c_str());
        }
        graph.resize(0);
        initialEstimate.clear();
        cout<<"-------- ---------------------------------- --------"<<endl;
    }
    else
    //SLAM输入初始化。
    //设置初始朝向角为0度，位置为0,0.以后的位姿以此为参考。
    {
        LOG(INFO)<<"Initializing slam factor."<<endl;
        noiseModel::Diagonal::shared_ptr priorNoise_Absolute = noiseModel::Diagonal::Sigmas(Vector3(0.3,0.3,0.1));
        graph.emplace_shared<PriorFactor<Pose2> >(Symbol('x',0),Pose2(0,0,0),priorNoise_Absolute);//位置和朝向角都初始化成0.
        noiseModel::Diagonal::shared_ptr priorNoise_Height = noiseModel::Diagonal::Sigmas(Vector2(1.0,0.0));
        graph.emplace_shared<PriorFactor<Point2> >(Symbol('h',0),Point2(0,0),priorNoise_Height);//高度初始化成0

        noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1,0);
        Rot2 rot_drift_prior = Rot2::fromAngle(0);
        //initialEstimate.insert(Symbol('y',0),...)//TODO.
        //graph.emplace_shared<PriorFactor<Rot2> >(Symbol('y',0),rot_drift_prior,priorNoise_Height);//定义初始SLAM yaw漂移角为0.
        double current_yaw_slam = get_yaw_from_slam_msg(SLAM_msg);

        LOG(INFO)<<"[DEBUG] Yaw in get_yaw_from_slam_msg:"<<current_yaw_slam*180/3.1415926<<endl;
        slam_vertex_index++;
    }
    //if gps-like measurement inputs:  (pay attention:here we use Point2 as input.)
    //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));
    //graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel)); //add observation
}

void GlobalOptimizationGraph::addBlockQRCode()
{
    //pEdgeQRCode = 
}
void addBlockSceneRetriever_StrongCoupling(); //Solve se(3) from multiple points PRXYZ;
void addBlockSceneRetriever_WeakCoupling();//just do square dist calc.

void GlobalOptimizationGraph::addBlockSceneRetriever()
{
    ;//TODO
    //step<1>.add vertex PR for scene.
    //set infomation matrix that optimize Rotation and altitude.Do not change longitude and latitude.
    //pBlockSceneRetriever = 
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
    //noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));
    //graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel));

    //when doing optimization:
    //  GaussNewtonParams parameters;
    //  GaussNewtonOptimizer optimizer(graph, initials, parameters);
    //  Values results = optimizer.optimize(); //get result.
    //  visualize loss function:(like 'chi2()')
    //  cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;

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
            noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
            graph.emplace_shared<BetweenFactor<Pose2> >(last_gps_vel_index,slam_vertex_index-1,Pose2(diff_x,diff_y,theta_slam_to_gps - yaw_init_to_gps),model_relative_movement);
        }
        last_gps_vel_index = slam_vertex_index-1;
    }
*/
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




//这些逻辑 放到StateTranfer里面去处理:
//    更新gps信息.
//    更新yaw角度.
//
//这段代码中用到的GlobalOptimizationGraph的内部变量:
//    yaw_init_to_gps
//    GPS_coord
//    graph //应该用不到.先放进去.
/*
    if(add_match_success && this->p_gps_slam_matcher->matchLen()>5)
    {
        bool yaw_calc_result_valid;
        double deg,deg_variance;
        p_gps_slam_matcher->check2IndexAndCalcDeltaDeg(0,p_gps_slam_matcher->matchLen()-1,//id
                                                       GPS_coord,yaw_calc_result_valid,deg,deg_variance
							);//尝试计算yaw.
        if(yaw_calc_result_valid)
        {
            LOG(INFO)<<"YAW UPDATED.New value:"<<deg<<" deg,covariance:"<<deg_variance<<" deg."<<endl;
            double _rad = (deg*3.1415926535)/180;
            this->yaw_init_to_gps = fix_angle(_rad);
        }
    }
    else
    {
        LOG(INFO)<<"Not in check func.Len:"<<this->p_gps_slam_matcher->matchLen()<<endl;
    }
    double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
    double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
    double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
    cout <<"setting gps measurement!"<<endl;
    cout <<"slam_vertex_index:"<<slam_vertex_index<<endl;

    Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
    bool covariance_valid = (GPS_msg.position_covariance_type >=2);
    double dx,dy;
    dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); // reproject to gog coordinate.
    dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps); 

    noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
    LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"    yaw:init to gps"<<yaw_init_to_gps<<endl;
    
    graph.add(GPSPose2Factor(Symbol('x',slam_vertex_index-1),//gtsam::Symbol('x',slam_vertex_index),
									 Point2(//gps_measurement_vec3d[0],gps_measurement_vec3d[1]
                                                                   dx,dy), gpsModel));
    cout <<"insert pos at "<<slam_vertex_index-1<<"."<<endl;
    cout <<"slam buffer size():"<<pSLAM_Buffer->size()<<endl;
    cout <<"graph.size()"<<graph.size()<<endl;
    last_gps_vertex_index = slam_vertex_index-1;
    cout<<"[GPS_INFO]GPS_relative_pos:"<<
            gps_measurement_vec3d[0]<<","<<
            gps_measurement_vec3d[1]<<","<<
            delta_alt<<endl;
*/
//整个都放进去,变成一个东西.这里加一个方法,允许StateTransfer从外面改里面的状态



#endif
