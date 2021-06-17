#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H



#include "GPS_SLAM_Matcher.h"
#include "StateTransfer.h"
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
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>

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
    void addBlockVelocity(int msg_index);//(const geometry_msgs::TwistStamped& velocity_msg);
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();
    
    //SLAM msg as edge prv and vertexspeed.
    std::deque<nav_msgs::Odometry> slam_prv_buffer;    
    void addSLAM_edgeprv(const geometry_msgs::PoseStamped& SLAM_msg);
    /*bool addGOGFrame(double x_guess,double y_guess)
    {
        GOG_Frame* pF = new GOG_Frame();
        cout <<"Insert "<<slam_vertex_index<<"in initialEstimate!"<<endl;
        initialEstimate.insert(slam_vertex_index,Pose2(
                                                    //initial guess of abs pos and yaw
                                                       x_guess,y_guess,0
							)); //here we use basic_vertex_id to represent vehicle position vertex id
        slam_vertex_index++;

    }*/
    bool doOptimization()
    {
        //bool retval = false;//if optimize success(edge >=3).
        //GaussNewtonParams parameters;
        //parameters.relativeErrorTol = 1e-5;// Stop iterating once the change in error between steps is less than this value
        //parameters.maxIterations = 10000;// Do not perform more than N iteration steps
        //GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);// Create the optimizer ...
       //Values result = optimizer.optimize();// ... and optimize
        return true;
    }

    void resetOptimizationGraph()
    {
    }
    void initBuffers(CallbackBufferBlock<geometry_msgs::PoseStamped> & SLAMbuf,
                     CallbackBufferBlock<sensor_msgs::NavSatFix>& GPSbuf,
                     CallbackBufferBlock<nav_msgs::Odometry>& AHRSbuf,
                     CallbackBufferBlock<geometry_msgs::TwistStamped>& Velocitybuf
    )
    {
        this->pSLAM_Buffer = &SLAMbuf;
        this->pGPS_Buffer = &GPSbuf;
        this->pVelocity_Buffer = &Velocitybuf;
        this->pAHRS_Buffer = &AHRSbuf;

        this->p_gps_slam_matcher = shared_ptr<GPS_SLAM_MATCHER>(new GPS_SLAM_MATCHER(this->pSLAM_Buffer,this->pGPS_Buffer,&(this->fSettings )) );
        this->p_state_tranfer_manager = shared_ptr<StateTransferManager>(new StateTransferManager(*p_gps_slam_matcher,this->fSettings,this->graph,this->GPS_coord,this->pGPS_Buffer,this->pSLAM_Buffer));
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


    NonlinearFactorGraph graph;
    Values initialEstimate;
    const int relinearizeInterval = 300;
    //NonlinearISAM* p_isam;//(relinearizeInterval);
    //NonlinearISAM* p_isam;
    ISAM2* p_isam;

    int status = STATUS_NO_GPS_NO_SCENE;

    int slam_vertex_index = 0;
    int last_gps_vertex_index = -1;
    int last_gps_vel_index = -1;
    double yaw_init_to_slam = 0.0; //GOG初始的y轴 转向 slam的y轴夹角.
    //double yaw_init_to_gps = 0.0; //GOG的y轴 到 gps的北 夹角.
    
    //message buffers.
    
    CallbackBufferBlock<geometry_msgs::PoseStamped> * pSLAM_Buffer;
    CallbackBufferBlock<sensor_msgs::NavSatFix> * pGPS_Buffer;
    CallbackBufferBlock<geometry_msgs::TwistStamped> * pVelocity_Buffer;
    CallbackBufferBlock<nav_msgs::Odometry> * pAHRS_Buffer;
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
};

GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{
    //cv::FileStorage fSettings;//(string(argv[1]),cv::FileStorage::READ);
    this->fSettings.open(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];

    //p_isam = new NonlinearISAM (relinearizeInterval);
    ISAM2Params isam2_params_; //在定義ISAM2例項的時候儲存引數的。
    isam2_params_.relinearizeThreshold = 0.01;
    isam2_params_.relinearizeSkip = 1;//多少个变量之后,relinearize.
    isam2_params_.enableRelinearization = false;//禁止relinearize.

    p_isam = new ISAM2(isam2_params_);
    
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
bool gps_msg_is_valid(const sensor_msgs::NavSatFix& gps)
{
    if(gps.status.status>=0 && gps.position_covariance_type>=2)
    {
        if( sqrt(pow(gps.position_covariance[0],2) + pow(gps.position_covariance[4],2)) < 30000//3
											 )
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
        GPS_coord.expandAt(GPS_msg.longitude,GPS_msg.latitude,GPS_msg.altitude);
        cout <<"Initiating GPS block in Optimization Graph!"<<endl;
    }
    /*if(this->allow_gps_usage == false || this->gps_init_success == false||  !(this->status&0x01)  )//check if GPS valid.
    {
        cout<<"[WARNING] Unable to add GPS edge.GPS usage is forbidden in config file."<<endl;
        if(this->allow_gps_usage)
        {
            cout<<"trying to reinit gps:"<<endl;
            this->try_reinit_gps();
        }
        return;
    }*/


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
        {
            //add pos and refine whole map.TODO.
            //step<1>.插入常规点约束.
            //double dx,dy;
            //int graph_node_index;
            //dx =  dy = .....
            //graph_node_index = ...
            //Point2 p2_input(dx,dy);
            //graph.add(GPSPose2Factor(graph_node_index,p2_input,gpsModel));
            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();

            bool init_yaw_valid_ = false;//好像gps这块得乘个-1...
            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);//这是和是否回环没关系的.
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);//这是和是否回环没关系的.

            cout <<"setting gps measurement!"<<endl;
            cout <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            double dx,dy;
            dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) + gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //     reproject to gog coordinate.
            dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) - gps_measurement_vec3d[0]*sin(yaw_init_to_gps);
            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
            LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;
            if(!init_yaw_valid_)//移到这里判断,以便于留下LOG.
            {
                LOG(WARNING)<<"init yaw not valid.can not add gps measurement"<<endl;
                return;
            }

 
            graph.add(GPSPose2Factor(slam_vertex_index-1,//gtsam::Symbol('x',slam_vertex_index),
                                                                             Point2(//gps_measurement_vec3d[0],    gps_measurement_vec3d[1]        
                                                                             dx,dy), gpsModel));
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

            graph.emplace_shared<BetweenFactor<Pose2> >(slam_vertex_index-1,slam_node_index_loop_,Pose2( diff_x,diff_y,diff_yaw),noise_model_relative_movement);
            LOG(INFO)<<"Loop mode GPS insertion step<2> finished."<<endl;
            //this->p_isam->update();Values currentEstimate = p_isam->calculateBestEstimate();

        }
        else
        {//常规操作.
            double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
            double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
            double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
            bool init_yaw_valid_ = false;
            //double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);
            //好像也是符号问题.
            double yaw_init_to_gps = this->p_state_tranfer_manager->getInitYawToWorldRad(init_yaw_valid_);
            cout <<"setting gps measurement!"<<endl;
            cout <<"slam_vertex_index:"<<slam_vertex_index<<endl;

            Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
            bool covariance_valid = (GPS_msg.position_covariance_type >=2);
            double dx,dy;
            dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) + gps_measurement_vec3d[1]*sin(yaw_init_to_gps); //     reproject to gog coordinate.
            dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) - gps_measurement_vec3d[0]*sin(yaw_init_to_gps);

            noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
            LOG(INFO) << "Adding gps measurement:"<<gps_measurement_vec3d[0]<<","<<gps_measurement_vec3d[1]<<endl<<"yaw:init to gps"<<yaw_init_to_gps<<endl;
            if(!init_yaw_valid_)//移到这里判断,方便产生LOG.
            {
                LOG(WARNING)<<"init yaw not valid.can not add gps measurement"<<endl;
                return;
            }

            graph.add(GPSPose2Factor(slam_vertex_index-1,//gtsam::Symbol('x',slam_vertex_index),
                                                                             Point2(//gps_measurement_vec3d[0],    gps_measurement_vec3d[1]
                                                                             dx,dy), gpsModel));
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
    
    graph.add(GPSPose2Factor(slam_vertex_index-1,//gtsam::Symbol('x',slam_vertex_index),
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
    Matrix3d R_SLAM_Mat = q_.toRotationMatrix();
    Vector3d vec_forward = R_SLAM_Mat*Vector3d(1,0,0);
    double fx,fy;//reproject to xOy;
    fx = vec_forward[0];fy = vec_forward[1];
    check_and_fix_dx(fx);
    double yaw = atan2(fy,fx);
    fix_angle(yaw);
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


void getDxDyFromSlamMsg(const geometry_msgs::PoseStamped& msg)
{
    
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
    GOG_Frame* pF = new GOG_Frame();
    cout <<"Insert "<<slam_vertex_index<<"in initialEstimate!"<<endl;
    this->p_state_tranfer_manager->updateSlam();
    initialEstimate.insert(slam_vertex_index,Pose2(//initial guess of abs pos and yaw
                                                   SLAM_msg.pose.position.x,SLAM_msg.pose.position.y,0
				)); //here we use basic_vertex_id to represent vehicle position vertex id

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


        double diff_yaw = (get_yaw_from_slam_msg(SLAM_msg) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)));
        //调调参数.
        //noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
        noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.001,0.001,0.0016));// 1mm/帧 0.1°/帧
        graph.emplace_shared<BetweenFactor<Pose2> >(slam_vertex_index-1,slam_vertex_index,Pose2( diff_x,diff_y,diff_yaw),model_relative_movement);
        slam_vertex_index++;

        p_isam->update(graph,initialEstimate);

        //FEJ实现:设置parameters.relinearizeSkip
        //ISAM2Params params_; //在定義ISAM2例項的時候儲存引數的。
	//  parameters.relinearizeThreshold = 0.01;
  	// parameters.relinearizeSkip = 1;

        Values currentEstimate = p_isam->calculateEstimate();//p_isam->estimate();
        LOG(INFO)<<"current yaw_init_to_slam:"<<yaw_init_to_slam*180/3.14159<<" deg."<<endl;
        cout <<"last state:"<<endl;
/*        Values result = DoglegOptimizer(graph, initialEstimate).optimize();

  //result.print("Final results:\n");

 
  for(Values::iterator key_value = result.begin(); key_value != result.end(); ++key_value)

  {

Key key = key_value->key;

Symbol asSymbol(key);

cout << asSymbol.chr() << std::endl;

cout << asSymbol.index() << std::endl;

 

if(asSymbol.chr() == 'l')

{

Point3 pt3 = key_value->value.cast<Point3>();

pt3.print("pt : ");

} else if(asSymbol.chr() == 'x')

{

Pose3 p3 = key_value->value.cast<Pose3>();

p3.print("pose3 : ");

 

Matrix4 mm = p3.matrix();

}

  }*/
        const Pose2* p_obj = &(currentEstimate.at(slam_vertex_index-1).cast<Pose2>());
                       //dynamic_cast<Pose2*>( &(currentEstimate.at(slam_vertex_index-1)) );//这几个值都不对,应该是内存错误.
                       //(Pose2*) (&p_isam->calculateEstimate(slam_vertex_index-1));
                       //dynamic_cast<Pose2> (currentEstimate.at(slam_vertex_index-1));
        LOG(INFO)<<"Current NODE ESTIMATED STATE at index:"<<slam_vertex_index-1<< " x:"<<p_obj->x()<<",y:"<<p_obj->y()<<",theta:"<<p_obj->theta()<<endl;
        LOG(INFO)<<"Current NODE ESTIMATED Position for visualizer:"<<p_obj->x()<<","<<p_obj->y()<<","<<p_obj->theta()*10<<endl;
        currentEstimate.print("Current estimate: ");
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
        graph.emplace_shared<PriorFactor<Pose2> >(0,Pose2(0,0,0),priorNoise_Absolute);//位置和朝向角都初始化成0.
        get_yaw_from_slam_msg(SLAM_msg);
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
#endif
