#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H

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
#include <opencv2/opencv.hpp>
#include "GOG_Frame.h"
#include "GPS_like.h"
using namespace std;
using namespace ygz;
using namespace gtsam;
const float thHuber = 50;

const int GPS_INIT_BUFFER_SIZE = 100;
using namespace ygz;
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

    void addBlockGPS(const sensor_msgs::NavSatFix& GPS_msg);
    void addBlockVelocity(const geometry_msgs::TwistStamped& velocity_msg);
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
        GaussNewtonParams parameters;
        parameters.relativeErrorTol = 1e-5;// Stop iterating once the change in error between steps is less than this value
        parameters.maxIterations = 10000;// Do not perform more than N iteration steps
        GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);// Create the optimizer ...
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
    void stateTransfer(int new_state);
private:
    NonlinearFactorGraph graph;
    Values initialEstimate;
    const int relinearizeInterval = 3;
    NonlinearISAM* p_isam;//(relinearizeInterval);

    int status = STATUS_NO_GPS_NO_SCENE;

    int slam_vertex_index = 0;
    int last_gps_vertex_index = -1;
    int last_gps_vel_index = -1;

    double yaw_init_to_slam = 0.0; //GOG初始的y轴 转向 slam的y轴夹角.
    double yaw_init_to_gps = 0.0; //GOG的y轴 到 gps的北 夹角.
    
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

    p_isam = new NonlinearISAM (relinearizeInterval);
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
        if( sqrt(pow(gps.position_covariance[0],2) + pow(gps.position_covariance[4],2)) < 3 )
        {
            return true;
        }
    }
    return false;
}

void GlobalOptimizationGraph::addBlockGPS(const sensor_msgs::NavSatFix& GPS_msg)
{
    if (!gps_msg_is_valid(GPS_msg))//unstable observation.
    {
        cout<<"GPS msg check failed.Return."<<endl;
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
    double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
    double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
    double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
    cout <<"setting gps measurement!"<<endl;

    Vector3d gps_measurement_vec3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),delta_alt);
    bool covariance_valid = (GPS_msg.position_covariance_type >=2);
    double dx,dy;
    dx = gps_measurement_vec3d[0]*cos(yaw_init_to_gps) - gps_measurement_vec3d[1]*sin(yaw_init_to_gps); // reproject to gog coordinate.
    dy = gps_measurement_vec3d[1]*cos(yaw_init_to_gps) + gps_measurement_vec3d[0]*sin(yaw_init_to_gps); 

    noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(GPS_msg.position_covariance[0], GPS_msg.position_covariance[4]));
    graph.add(GPSPose2Factor(gtsam::Symbol('x',slam_vertex_index), Point2(//gps_measurement_vec3d[0],gps_measurement_vec3d[1]
                                                                   dx,dy), gpsModel));
    last_gps_vertex_index = slam_vertex_index;
    cout<<"[GPS_INFO]GPS_relative_pos:"<<
            gps_measurement_vec3d[0]<<","<<
            gps_measurement_vec3d[1]<<","<<
            delta_alt<<endl;
}


const double EPS = 0.0001;
const double PI = 3.1415926535;
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

void GlobalOptimizationGraph::addBlockVelocity(const geometry_msgs::TwistStamped& velocity_msg)
{
    if(last_gps_vel_index == -1)
    {
        last_gps_vel_index = slam_vertex_index;
    }
    else
    {
        //计算差.
        double vx = velocity_msg.twist.linear.x;
        double vy = velocity_msg.twist.linear.y;
        double dt = pSLAM_Buffer->at(slam_vertex_index).header.stamp.toSec()- pSLAM_Buffer->at(last_gps_vel_index).header.stamp.toSec();
        double dx_vel = vx*dt;
        double dy_vel = vy*dt;
        double dx_slam = this->pSLAM_Buffer->at(slam_vertex_index).pose.position.x - this->pSLAM_Buffer->at(last_gps_vel_index).pose.position.x;
        double dy_slam = this->pSLAM_Buffer->at(slam_vertex_index).pose.position.y - this->pSLAM_Buffer->at(last_gps_vel_index).pose.position.y;
        //计算yaw.两种方法：几何法，最小二乘重投影。
        check_and_fix_dx(dx_slam); //避免出现极小。
        check_and_fix_dx(dx_vel);
        double theta_slam = atan2(dy_slam,dx_slam);
        double theta_gps = atan2(dy_vel,dx_vel);
        double theta_slam_var = calc_angle_variance(dx_slam,dy_slam,dx_slam_var,dy_slam_var);//dx_slam_var,dy_slam_var填入slam pos差分的标准差.
        double theta_gps_var = calc_angle_variance(dx_vel,dy_vel,vel_x_var,vel_y_var);

        //先计算theta_slam_to_gps;再变换到theta_init_to_gps;
        double theta_slam_to_gps = theta_gps - theta_slam;
        double yaw_init_to_gps = yaw_init_to_slam - theta_slam_to_gps;
        fix_angle(yaw_init_to_gps);

        //sin(yaw_init_to_gps) cos(yaw_init_to_gps)
        double diff_x = cos(-1*yaw_init_to_gps)*dx_vel - sin(-1*yaw_init_to_gps)*dy_vel;
        double diff_y = sin(-1*yaw_init_to_gps)*dy_vel + cos(-1*yaw_init_to_gps)*dx_vel;

        if(calcnorm(dx_slam,dy_slam) > 0.1 && calcnorm(dx_vel,dy_vel)>0.1) //认为已经观测到有效的yaw.
        {
            //diff_yaw = yaw_init_to_gps - last_yaw_init_to_gps; // last_yaw_init_to_gps //这个应该不产生yaw的观测，因为机身的转动不可观测。
            noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
            graph.emplace_shared<BetweenFactor<Pose2> >(last_gps_vel_index,slam_vertex_index,Pose2(diff_x,diff_y,theta_slam_to_gps - yaw_init_to_gps),model_relative_movement);
            last_gps_vel_index = slam_vertex_index;
        }
    }
}

void GlobalOptimizationGraph::addBlockSLAM(int msg_index)//(const geometry_msgs::PoseStamped& SLAM_msg)
//if use this api,only reserve the last one.
//void GlobalOptimizationGraph::addBlockSLAM(std::vector<const geometry_msgs::PoseStamped&> SLAM_msg_list)
//for multiple msgs.
{
    //两种策略：1.来slam消息就立即创建节点
    //          2.等到gps信息到来再创建。
    //这里我们选择第2种。
    auto SLAM_msg = pSLAM_Buffer->at(msg_index);
    //addGOGFrame(SLAM_msg.pose.position.x,SLAM_msg.pose.position.y);//create a new map 'vertexPR'
    GOG_Frame* pF = new GOG_Frame();
    cout <<"Insert "<<slam_vertex_index<<"in initialEstimate!"<<endl;
    initialEstimate.insert(slam_vertex_index,Pose2(//initial guess of abs pos and yaw
                                                   SLAM_msg.pose.position.x,SLAM_msg.pose.position.y,0
				)); //here we use basic_vertex_id to represent vehicle position vertex id

    if (slam_vertex_index >0)
    {
        //calc diff
        auto orient = SLAM_msg.pose.orientation;
        Eigen::Quaterniond q_;
        q_.x() = orient.x;q_.y() = orient.y;q_.z() = orient.z;q_.w() = orient.w;
        Matrix3d R_SLAM_Mat = q_.toRotationMatrix();
        double fx,fy;//reproject to xOy;
        Vector3d vec_forward = R_SLAM_Mat*Vector3d(1,0,0);
        fx = vec_forward[0];fy = vec_forward[1];
        double diff_x,diff_y;
        diff_x = fx*cos(yaw_init_to_slam) - fy*sin(yaw_init_to_slam);
        diff_y = fy*cos(yaw_init_to_slam) + fx*cos(yaw_init_to_slam);

        double diff_yaw = (get_yaw_from_slam_msg(SLAM_msg) - get_yaw_from_slam_msg(this->pSLAM_Buffer->at(slam_vertex_index-1)));
        noiseModel::Diagonal::shared_ptr model_relative_movement = noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));//for slam diff rotation and translation.
        graph.emplace_shared<BetweenFactor<Pose2> >(slam_vertex_index-1,slam_vertex_index,Pose2( diff_x,diff_y,diff_yaw),model_relative_movement);
        slam_vertex_index++;

        p_isam->update(graph,initialEstimate);
        Values currentEstimate = p_isam->estimate();
        currentEstimate.print("Current estimate: ");
        graph.resize(0);
        initialEstimate.clear();

        cout<<"-------- ---------------------------------- --------"<<endl;
    }
    else//init
    {
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
