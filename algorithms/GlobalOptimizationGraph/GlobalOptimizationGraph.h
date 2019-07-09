
#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimizable_graph.h>
#include "G2OTypes.h"
#include "G2OTypes_EdgeSLAMPRV.h"
#include <opencv2/core/persistence.hpp>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "GPSExpand.h"
#include "CallbacksBufferBlock.h"
#include <cmath>
#include <deque>
#include <opencv2/opencv.hpp>
#include "GOG_Frame.h"
using namespace std;
using namespace ygz;
//采用自顶向下设计方法,分块逐步实现.
//先实现AHRS+GPS.
//测试.
//再支持SceneRetrieve.重点是实现3个scene适配变换.
//最后加入SLAM和速度估计部分,实现优化图.


//约定坐标系:全局坐标使用NED坐标系.
//局部坐标系采用机身坐标系.SLAM和回环对齐到这个坐标系.


//struct State//the prev-state of drone.
//{
//    
//};
const float thHuber = 50;
struct PRState
{
    Matrix3d rotation;
    Vector3d translation;
    Vector3d& t()
    {
        return this->translation;
    }
    Matrix3d& R()
    {
        return this->rotation;
    }
};
typedef VertexPR State;
typedef VertexSpeed Speed;

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
    inline bool isWorkingWithGPS()
    {
        return(this->allow_gps_usage&& this->gps_init_success);
    }
    void addBlockAHRS(const nav_msgs::Odometry& AHRS_msg
        
    );
    //SLAM msg as fixed vertexpr node.
    void addBlockSLAM(const geometry_msgs::PoseStamped& SLAM_msg);
    

    void addBlockGPS(const sensor_msgs::NavSatFix& GPS_msg);
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();
    
    //SLAM msg as edge prv and vertexspeed.
    std::deque<nav_msgs::Odometry> slam_prv_buffer;    
    void addSLAM_edgeprv(const geometry_msgs::PoseStamped& SLAM_msg);
    void clearSLAM_edgeprv_buffer()
    {
        this->slam_prv_buffer.clear();
    }
    
    bool addGOGFrame()
    {
        GOG_Frame* pF = new GOG_Frame();
        pF->pPRVertex = new VertexPR();
        pF->pSpeedVertex = new VertexSpeed();
        int basic_vertex_id = this->iterateNewestFrameID();
        pF->pPRVertex->setId(basic_vertex_id+0);
        pF->pSpeedVertex->setId(basic_vertex_id+1);
        this->optimizer.addVertex(pF->pPRVertex);
        this->optimizer.addVertex(pF->pSpeedVertex);
        this->SlidingWindow.push_front(*pF);
        const int MAX_WIND_SIZE = 10;
        if(this->SlidingWindow.size()>MAX_WIND_SIZE)
        {//window management
            this->SlidingWindow.back().pPRVertex->setFixed(1);
            this->SlidingWindow.back().pSpeedVertex->setFixed(1);
            this->SlidingWindow.pop_back();
        }
        
    }
    bool doOptimization()
    {
        //When DEBUG:
        bool retval = false;//if optimize success(edge >=3).
        this->debug_show_optimization_graph();//just show how this optimizable_graph looks like.
        
         //TODO change to <3 for gps-denied condition.
        if(this->optimizer.edges().size()<=3)
        {
            retval = false;
            this->resetOptimizationGraph();
            cout<<"[OPTIMIZER_INFO] error in doOptimization.quit."<<endl;
            return retval;
        }
        retval = true;
        this->optimizer.initializeOptimization();
        this->optimizer.optimize(10);
        /*this->historyStatesWindow.push_front(currentState);
        //this->historyStatesWindow.reserve();///...
        */
        //clear optimizer and reset!

        double chi2 = this->optimizer.chi2();

        if(chi2 > 10000)
        {
            cout<<"[OPTIMIZER_WARNING] chi2 is:"<<chi2<<",chi2 >10000，Optimizer result not stable."<<endl;
            //this->resetOptimizationGraph();
            //return retval;

        }
        cout<<"[OPTIMIZER_INFO] chi2 is:"<<chi2<<endl;
        if(this->optimizer.vertex(this->newest_frame_id+0) == NULL)
        {
            cout<<"[OPTIMIZER_INFO] Error.Position vertex is NULL!"<<endl;
        }
        
        this->last_position = dynamic_cast<ygz::VertexPR *>(this->optimizer.vertex(this->newest_frame_id+0))->t();
        cout<<"[OPTIMIZER_INFO] Last_pos_of_optimizer:"<<this->last_position[0]<<","<<this->last_position[1]
                                                        <<","<<this->last_position[2]<<endl;

        Vector3d velo = (dynamic_cast<VertexSpeed*>  (this->optimizer.vertex(this->newest_frame_id+1)) )->estimate();
        cout<<"[OPTIMIZER_INFO] speed:"<<velo[0]<<","<<velo[1]<<","<<velo[2]<<endl;
        /*
        //this->last_orientation = dynamic_cast<ygz::VertexPR *>(this->optimizer.vertex(this->newest_frame_id+0))->R();
        //cout<<"[OPTIMIZER_INFO] Last_orientation_of_optimizer:\n"<<this->last_orientation<<endl;
        */
        
        //this->resetOptimizationGraph();
        cout<<"DEBUG:end of do optimization():"<<endl;
        this->debug_show_optimization_graph();
        return retval;
    }

    void resetOptimizationGraph()
    {
        this->optimizer.clear();
        this->resetNewestFrameID();
        //this->optimizer = g2o::SparseOptimizer();
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
        solver_ptr = new g2o::BlockSolverX(linearSolver);
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);//reset to init state.
        //VertexPR* pcurrent_state = new VertexPR;
        //pcurrent_state->setId(this->getNewestVertexSpeedID());
        //optimizer.addVertex(pcurrent_state);
        
    }
    bool SpeedInitialization();
    bool estimateCurrentSpeed();
    void initBuffers(CallbackBufferBlock<geometry_msgs::PoseStamped> & SLAMbuf,
                     CallbackBufferBlock<sensor_msgs::NavSatFix>& GPSbuf,
                     CallbackBufferBlock<nav_msgs::Odometry>& AHRSbuf
    )
    {
        this->pSLAM_Buffer = &SLAMbuf;
        this->pGPS_Buffer = &GPSbuf;
        this->pAHRS_Buffer = &AHRSbuf;
    }
    shared_ptr<VertexPR> getpCurrentPR()
    {
        return this->pCurrentPR;
    }
    void updateSLAM_AHRS_relative_rotation_translation()
    {
        if(this->pSLAM_Buffer->size()<1 || this->pAHRS_Buffer->size()<1)
        {
            cout<<"[RELATIVE_ROTATION_INFO] SLAM or AHRS buffer size <1.Cannot match rotation mat."<<endl;
            return;
        }
        //match rotation.
            //1.get slam msg.
        auto slam_msg = this->pSLAM_Buffer->getLastMessage();
        auto q = slam_msg.pose.orientation;//TODO
        Eigen::Quaterniond q_;
        q_.w() = q.w;
        q_.x() = q.x;
        q_.y() = q.y;
        q_.z() = q.z;
        auto t_slam = slam_msg.pose.position;
        Vector3d t_slam_(t_slam.x,t_slam.y,t_slam.z);

        Matrix3d prev_transform_inv;
        //prev_transform_inv << 0,0,-1,1,0,0,0,1,0;
        prev_transform_inv << 0,0,-1,1,0,0,0,-1,0;
        //prev_transform = prev_transform.inverse();
        Matrix3d prev_t = prev_transform_inv.inverse();
            //2.get ahrs msg.
        /*auto AHRS_msg = this->pAHRS_Buffer->getLastMessage();//直接计算 SLAM和AHRS差距太大，怀疑是AHRS漂移。。。。
        auto q2 = AHRS_msg.pose.pose.orientation;
        auto q2;
        Eigen::Quaterniond q_ahrs;
        q_ahrs.w() = q2.w;
        q_ahrs.x() = q2.x;
        q_ahrs.y() = q2.y;
        q_ahrs.z() = q2.z;
        Matrix3d R_ahrs = q_ahrs.toRotationMatrix();
        */
        
        //Matrix3d R_ahrs = this->last_orientation;//迭代求解
        //SLAM_to_UAV_coordinate_transfer_R = (q_.toRotationMatrix()*prev_t).inverse()* (R_ahrs);
        //match translation.//TODO.
        //SLAM_to_UAV_coordinate_transfer_t =  -1* (this->SLAM_to_UAV_coordinate_transfer_R* t_slam_) + this->last_position;
    }
private:
    void debug_show_optimization_graph()
    {
        cout<<"Size of vertices:"<<this->optimizer.vertices().size()<<endl;//size of vertices.
        cout<<"Size of Edges:"<<this->optimizer.edges().size()<<endl;//size of edges.
    }
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
    int status = STATUS_NO_GPS_NO_SCENE;
    
    //message buffers.
    
    CallbackBufferBlock<geometry_msgs::PoseStamped> * pSLAM_Buffer;
    CallbackBufferBlock<sensor_msgs::NavSatFix> * pGPS_Buffer;
    CallbackBufferBlock<nav_msgs::Odometry> * pAHRS_Buffer;
    //uav location attitude info management.
    

    deque<GOG_Frame> SlidingWindow;
    //std::deque<State> historyStatesWindow;
    State currentState;

    Speed currentSpeed;
    std::deque<Speed> historySpeed;
    void resetSpeed();
    /*
    void marginalizeAndAddCurrentFrameToHistoryState()
    {
        int max_window_size = (this->fSettings)["OPTIMIZATION_GRAPH_KF_WIND_LEN"];
        if(this->historyStatesWindow.size() > max_window_size)
        {
            State p = historyStatesWindow.back();
            //p.setFixed(1);//set this fixed. remove it from optimizable graph.
            this->historyStatesWindow.pop_back();//delete last one.
        }
        //TODO:fix speed window. the length of speed window shall be ( len(position) - 1).
        this->historyStatesWindow.push_front(this->currentState);
        //TODO: how do we set new state?
        State new_state;
        this->currentState = new_state;
    }*/
    /*
    void autoInsertSpeedVertexAfterInsertCurrentVertexPR(const Vector3d & current_fixed_slam_position)
    {
        if(this->SlidingWindow.size()<1)
        {
            return;//nothing to do.
        }
        Speed* pNewSpeed = new Speed();
        State s = currentState;
        
        pNewSpeed->setEstimate(s.t() - this->SlidingWindow.front().pPRVertex->t());
        this->optimizer.addVertex(pNewSpeed);
        
        EdgeSLAMPRV* pNewEdgePRV = new EdgeSLAMPRV;
        //pEdgePRV->setMeasurement();//TODO:set slam info into this edge.
        //pEdgePRV->setVertex(0,pNewEdgePRV);
        
    }*/
    int frameID = 0;
    int newest_frame_id = 0;
    int iterateNewestFrameID()
    {
        int retval = this->frameID;
        
        this->frameID+=2;
        //this->frameID++;//using VertexPR only.
        this->newest_frame_id = retval;
        return retval;
    }
    int resetNewestFrameID()
    {
        int retval = this->frameID;
        this->frameID = 0;
        return retval;
    }
    int getNewestVertexSpeedID()
    {
        int retval = this->frameID;
        this->frameID+=2;
        return retval;
    }
    
    //SLAM
    //VertexPR SLAM_to_UAV_coordinate_transfer;
    Matrix3d SLAM_to_UAV_coordinate_transfer_R;
    Vector3d SLAM_to_UAV_coordinate_transfer_t;
    //AHRS
    Matrix3d ahrs_R_init;

    //history:
    Matrix3d last_orientation;
    Vector3d last_position;


    //optimization graph.
    //G2OOptimizationGraph graph;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg *solver;

    //vector<shared_ptr<g2o::optimizable_graph::Vertex> > VertexVec;
    //vector<shared_ptr<g2o::optimizable_graph::Edge> > EdgeVec;

    shared_ptr<VertexPR> pCurrentPR;
    //gps configuration and initiation.
    void remap_UAV_coordinate_with_GPS_coordinate();
    GPSExpand GPS_coord;

    bool allow_gps_usage = true;//can be configured by config.yaml
    bool gps_init_success = false;
    double gps_init_longitude,gps_init_latitude,gps_init_altitude;

    double gps_init_lon_variance,gps_init_lat_variance,gps_init_alt_variance;
    std::vector<sensor_msgs::NavSatFix> gps_info_buffer;
    //scene part.
    void fix_scene_Rotation_with_AHRS();
    void remap_scene_coordinate_with_GPS_coordinate();
    void remap_scene_coordinate_with_UAV_coordinate();
    void try_reinit_gps()
    {
        ;
    }
    VertexPR relative_scene_to_UAV_body;
    
    //time for calc speed vertex
    double last_slam_msg_time;
    double init_slam_msg_time;
};

GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{
    //cv::FileStorage fSettings;//(string(argv[1]),cv::FileStorage::READ);
    this->fSettings.open(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //this->optimizer = g2o::SparseOptimizer();
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);


    this->last_position[0] = 0;
    this->last_position[1] = 0;
    this->last_position[2] = 0;
    this->last_orientation = Eigen::Matrix<double,3,3>::Identity();
    //optimizer.setVertex();
    /*currentState.setEstimate(currentState);
    currentState.setId(0);//vertex 0 in optimization graph.*/
    //currentState.setId(0);
    //optimizer.addVertex(&currentState);
}
bool GlobalOptimizationGraph::init_AHRS(const nav_msgs::Odometry& AHRS_msg)
{
    auto q = AHRS_msg.pose.pose.orientation;
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    Matrix3d R_init = q_.toRotationMatrix();
    this->ahrs_R_init = R_init;
    //TODO:set R_init into graph state.
    this->currentState.R() = R_init;
    this->last_orientation = R_init;
    return true;
}
bool GlobalOptimizationGraph::init_SLAM(//const geometry_msgs::PoseStamped& slam_msg
)
{
    cout<<"In init_SLAM()."<<endl;
    //match rotation matrix and translation vector to E,0.
    auto slam_msg = this->pSLAM_Buffer->getLastMessage();
    auto q = slam_msg.pose.orientation;//TODO
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    auto t_slam = slam_msg.pose.position;
    Vector3d t_slam_(t_slam.x,t_slam.y,t_slam.z);
    //Matrix3d flu_to_enu1,flu_to_enu2;
    //flu_to_enu1 << 0,1,0,-1,0,0,0,0,1;
    //flu_to_enu2 << 1,0,0,0,0,1,0,-1,0;
    
    
    //SLAM_to_UAV_coordinate_transfer_R = (q_.toRotationMatrix()*flu_to_enu1*flu_to_enu2).inverse() *this->ahrs_R_init;
    Matrix3d prev_transform;
    //prev_transform<<1,0,0,0,-1,0,0,0,-1;
    //prev_transform<<1,0,0,0,1,0,0,0,-1;
    //prev_transform<<-1,0,0,0,1,0,0,0,1;

    prev_transform << 0,0,-1,1,0,0,0,1,0;
    //prev_transform << 0,0,-1,1,0,0,0,-1,0;
    //prev_transform = prev_transform.inverse();
    Matrix3d prev_t = prev_transform.inverse();
/*
    SLAM_to_UAV_coordinate_transfer_R = (q_.toRotationMatrix()
                                                //*prev_t
                                            ).inverse()* (this->ahrs_R_init);
*/
    SLAM_to_UAV_coordinate_transfer_R = this->ahrs_R_init;
    Matrix3d finaltransform;
    //finaltransform<<0,0,1,1,0,0,0,-1,0;
    
    
    //finaltransform<<0,0,1,-1,0,0,0,1,0;
    //finaltransform<<-1,0,0, 0,0,-1, 0,1,0;//rotate 90deg y.
    finaltransform<<0,0,-1,1,0,0,0,1,0;
    
    
    //finaltransform<<0,0,-1,-1,0,0,0,-1,0;
    //SLAM_to_UAV_coordinate_transfer_R = SLAM_to_UAV_coordinate_transfer_R*finaltransform;
    
    
    SLAM_to_UAV_coordinate_transfer_t = -1 * SLAM_to_UAV_coordinate_transfer_R * t_slam_;
    
    //int vinit_pr_id = this->iterateNewestFrameID();
    //if(vinit_pr_id!=0)
    //{
    //    cout<<"ERROR:vinit_pr_id!=0!"<<endl;
    //    return false;
    //}
    /*
    this->currentState.R() = this->ahrs_R_init;
    this->currentState.t() = Vector3d(0,0,0);
    this->currentState.setId(vinit_pr_id);
    this->currentState.setFixed(true);
    */
    //VertexPR* pcurrent_state = new VertexPR();
    cout<<"operate SlidingWindow.front()."<<endl;
    addGOGFrame();
    this->SlidingWindow.front().slam_frame_time = slam_msg.header.stamp.toSec();//set timestamp.
    this->SlidingWindow.front().SLAM_msg = slam_msg;
    VertexPR* pcurrent_state = this->SlidingWindow.front().pPRVertex;
    pcurrent_state->R() = this->ahrs_R_init;
    pcurrent_state->t() = Vector3d(0,0,0);
    //pcurrent_state->setId(0);
    pcurrent_state->setFixed(true);
    
    
    //estimate speed: 0 0 0 
    this->SlidingWindow.front().pSpeedVertex->setEstimate(Vector3d(0,0,0));

    //this->optimizer.addVertex(pcurrent_state);
    cout<<"[SLAM_INFO] SLAM_init_R:\n"<<q_.toRotationMatrix()<<endl;
    cout<<"[SLAM_INFO] ahrs_R:\n"<<this->ahrs_R_init<<endl;
    cout<<"[SLAM_INFO] SLAM_to_UAV_coordinate_transfer R:\n"<<SLAM_to_UAV_coordinate_transfer_R<<endl;
    cout<<"[SLAM_INFO] slam init finished!!!"<<endl;
    return true;
}

bool GlobalOptimizationGraph::init_gps()//init longitude,latitude,altitude.
    //TODO:Init a GPS callback buffer block class,inherit callback buffer base,implement init and check avail.
{
    cout<<"In GOG::init_gps(): trying to init gps."<<endl;
    double avg_lon,avg_lat,avg_alt;
	avg_lon = 0;
	avg_lat = 0;
	avg_alt = 0;
	int count = 0;
	//assert gaussian distribution,sigma>3 is rejection region.
	double vari_lon,vari_lat,vari_alt;
	vari_lon=0;
	vari_lat=0;
	vari_alt=0;
    
    //for(auto g:*(this->pGPS_Buffer))
    auto buf = this->pGPS_Buffer->getCopyVec();
    for(auto g: buf)
    {
        if(g.status.status>=g.status.STATUS_FIX) //which means gps available.
        {
            avg_lon+=g.longitude;
            avg_lat+=g.latitude;
            avg_alt+=g.altitude;
            count++;
            
        }
        else
        {
            cout<<"MSG status:"<<(int)g.status.status<<" is smaller than g.status.STATUS_FIX."<<endl;
            cout<<"info: lon lat alt:"<<g.longitude<<","<<g.latitude<<","<<g.altitude<<endl;
            
            //TODO:remove this.DEBUG ONLY!!!
            cout<<"WARNING: NO CHECK ON gps status!"<<endl;
            avg_lon+=g.longitude;
            avg_lat+=g.latitude;
            avg_alt+=g.altitude;
            count++;
        }
    }
	if(count<GPS_AVAIL_MINIMUM)
	{
        cout<<"In GOG::init_gps(): count < GPS_AVAIL_MINIMUM.count:"<<count<<"."<<endl;
	    return false;
	}
	avg_lon = avg_lon/count;
	avg_lat = avg_lat/count;
	avg_alt = avg_alt/count;
	for(auto g:buf)
	{
	    //if(g.status.status>=g.status.STATUS_FIX)
	    if(true)//TODO:debug only!
        {
	        vari_lon+=pow(g.longitude-avg_lon,2);
			vari_lat+=pow(g.latitude-avg_lat,2);
			vari_alt+=pow(g.altitude-avg_alt,2);
	    }
	}
	vari_lon/=count;
	vari_lat/=count;
	vari_alt/=count;
    vari_lon = sqrt(vari_lon);
    vari_lat = sqrt(vari_lat);
    vari_alt = sqrt(vari_alt);
	cout<<"[GPS_INFO]GPS Initiated at LONGITUDE:"<<avg_lon<<",LATITUDE:"<<avg_lat<<",ALTITUDE:"<<avg_alt<<".VARIANCE:"<<vari_lon<<", "<<vari_lat<<", "<<vari_alt<<"."<<endl;
	cout<<"Available count:"<<count<<"."<<endl;
	
	//expand at avg lon,lat.
	GPSExpand GE;
	GE.expandAt(avg_lon,avg_lat,avg_alt);
    this->GPS_coord.expandAt(avg_lon,avg_lat,avg_alt);
    double lon_variance_m,lat_variance_m;
    lon_variance_m = GE.vari_km_per_lon_deg()*vari_lon*1000;
    lat_variance_m = GE.vari_km_per_lat_deg()*vari_lat*1000;

	cout<<"[GPS_INFO] X variance:"<<GE.vari_km_per_lon_deg()*vari_lon*1000<<"m;Y variance:"
            <<GE.vari_km_per_lat_deg()*vari_lat*1000<<
            "m,ALTITUDE variance:"<<vari_alt<<"m."<<endl;
    //check variance:
    double gps_init_variance_thres = (this->fSettings)["GPS_INIT_VARIANCE_THRESHOLD_m"];
    double gps_init_alt_variance_thres =  (this->fSettings)["GPS_INIT_ALT_VARIANCE_THRESHOLD_m"];
    cout<<"Config file read."<<endl;
    if(lon_variance_m> gps_init_variance_thres || lat_variance_m > gps_init_variance_thres
        || vari_alt>gps_init_alt_variance_thres)
    {
        cout<<"WARNING:GPS init failed.VARIANCE out of threshold."<<endl;
        cout<<"THRESHOLD(m):"<<gps_init_variance_thres<<endl;
        return false;
    }
    else
    {
        cout<<"gps variance check passed!"<<endl;
    }
    this->gps_init_longitude = avg_lon;
    this->gps_init_latitude = avg_lat;
    this->gps_init_altitude = avg_alt;

    this->gps_init_lon_variance = vari_lon;
    this->gps_init_lat_variance = vari_lat;
    this->gps_init_alt_variance = vari_alt;

    gps_init_success = true;
    if ( !(this->status&1)) // gps_avail 
    {
        this->status |= 1;//set status.
    }
	return true;
}
void GlobalOptimizationGraph::addBlockAHRS(const nav_msgs::Odometry& AHRS_msg)
{
    cout<<"Adding AHRS block to Optimization Graph!"<<endl;
    EdgeAttitude* pEdgeAttitude = new EdgeAttitude();
    auto q = AHRS_msg.pose.pose.orientation;//TODO
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    //pEdgeAttitude->setMeasurement(q_);//TODO
    const double weight_ahrs = 0.01;
    Eigen::Matrix<double,3,3> info_mat = weight_ahrs * Eigen::Matrix<double,3,3>::Identity();
    pEdgeAttitude->setInformation(info_mat);
    pEdgeAttitude->setLevel(!checkAHRSValid());
    cout<<"adding vertex ahrs."<<endl;
    
    //pEdgeAttitude->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));
    int newest_vpr_id = this->newest_frame_id + 0;
    if(this->optimizer.vertex(newest_vpr_id) == NULL)
    {
        cout<<"error:(in addBlockAHRS() ) optimizer.vertex("<<newest_vpr_id<<") is NULL!"<<endl;
    }
    pEdgeAttitude->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *> (this->optimizer.vertex(newest_vpr_id)) );
    cout<<"added vertex ahrs."<<endl;
    const bool AHRS_ROTATION_USE_ROBUST_KERNEL = true;
    if(AHRS_ROTATION_USE_ROBUST_KERNEL)
    {
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        pEdgeAttitude->setRobustKernel(rk);
        rk->setDelta(thHuber);
    }



    //cout<<"WARNING:EDGE AHRS NOT ADDED!!"<<endl;

    this->optimizer.addEdge(pEdgeAttitude);
    cout<<"AHRS EDGE ADDED!"<<endl;

}
void GlobalOptimizationGraph::addBlockGPS(const sensor_msgs::NavSatFix& GPS_msg)
{
    cout <<"Adding GPS block to Optimization Graph!"<<endl;
    
    double gps_max_delay_sec = this->fSettings["GPS_MAX_DELAY_SEC"];
    double delay_sec = this->pGPS_Buffer->queryLastMessageTime() - GPS_msg.header.stamp.toSec();
    if(this->pGPS_Buffer->queryLastMessageTime() - GPS_msg.header.stamp.toSec() > gps_max_delay_sec)
    {
        cout<<"[INFO]GPS msg time out.Delay:"<<delay_sec<<"s."<<endl;
        
        //set gps invalid. if possible,try reinit.
        this->status&=(0x02);
        //this->gps_init_success = false;
    }

    if(this->allow_gps_usage == false || this->gps_init_success == false||  !(this->status&0x01)  )//check if GPS valid.
    {
        cout<<"[WARNING] Unable to add GPS edge.GPS usage is forbidden in config file."<<endl;
        if(this->allow_gps_usage)
        {
            cout<<"trying to reinit gps:"<<endl;
            this->try_reinit_gps();
        }
        return;
    }
    /*//state shall be put into state tranfer module.
    if(!(this->status&this->STATUS_WITH_GPS_NO_SCENE))
    {
        //match with new coordinate
        this->GPS_coord.init_at(xxx_msg);
    }*/


  
    EdgePRGPS* pEdgePRGPS = new EdgePRGPS();
    double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
    double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
    double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();
    cout <<"setting gps measurement!"<<endl;

    //since the quaternion is NED defined,we do not need any rotation here.
    pEdgePRGPS->setMeasurement( Vector3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),
                                delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),
                                delta_alt)
                            );

    double info_lon,info_lat,info_alt;
    double gps_min_variance_lonlat_m = (this->fSettings)["GPS_MIN_VARIANCE_LONLAT_m"];
    double gps_min_variance_alt_m = (this->fSettings)["GPS_MIN_VARIANCE_ALT_m"];
    cout <<"in addGPSBlock(): calc info mat!"<<endl;
  info_lon = min((1.0/this->gps_init_lon_variance),1.0/gps_min_variance_lonlat_m);
  info_lat = min((1.0/this->gps_init_lat_variance),1.0/gps_min_variance_lonlat_m);
  info_alt = min((1.0/this->gps_init_alt_variance),1.0/gps_min_variance_alt_m);

  Eigen::Matrix<double,3,3> info_mat = Eigen::Matrix<double,3,3>::Identity();
  info_mat(0,0) = info_lon;
  info_mat(1,1) = info_lat;
  info_mat(2,2) = info_alt;
  pEdgePRGPS->setInformation(info_mat);//the inverse mat of covariance.
  int gps_valid = (GPS_msg.status.status >= GPS_msg.status.STATUS_FIX?0:1);
  if(gps_valid!=0)
  {
      cout<<"[GPS_INFO]:GPS lock failed."<<endl;
  }
  pEdgePRGPS->setLevel(gps_valid);
  cout<<"adding edge gps!state:"<<bool(gps_valid==0)<<endl;
  cout<<"[GPS_INFO]GPS_relative_pos:"<<
            delta_lon*1000*GPS_coord.vari_km_per_lon_deg()<<","<<
            delta_lat*1000*GPS_coord.vari_km_per_lat_deg()<<","<<
            delta_alt<<endl;
  cout<<"[GPS_INFO] info mat:\n"<<info_mat<<endl;
  
  int newest_vpr_id = this->newest_frame_id + 0;
  if(this->optimizer.vertex(newest_vpr_id) == NULL)
  {
      cout<<"error:(in addBlockGPS() ) optimizer.vertex("<<newest_vpr_id<<") is NULL!"<<endl;
  }
  pEdgePRGPS->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(newest_vpr_id)));

  const bool GPS_USE_ROBUST_KERNEL = true;
  if(GPS_USE_ROBUST_KERNEL)
  {
      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      pEdgePRGPS->setRobustKernel(rk);
      rk->setDelta(thHuber);
  }
  this->optimizer.addEdge(pEdgePRGPS);
}

bool GlobalOptimizationGraph::estimateCurrentSpeed()
{
  //step<1> check status change log;
  //step<2> if status not changed;calc current speed.
  //step<3> set into vector.
}


void GlobalOptimizationGraph::addBlockSLAM(const geometry_msgs::PoseStamped& SLAM_msg)
//if use this api,only reserve the last one.
//void GlobalOptimizationGraph::addBlockSLAM(std::vector<const geometry_msgs::PoseStamped&> SLAM_msg_list)
//for multiple msgs.
{
    //addGOGFrame(SLAM_msg.header.stamp.toSec());//called in ROS IO Manager.
    //set timestamp
    this->SlidingWindow.front().slam_frame_time = SLAM_msg.header.stamp.toSec();
    this->SlidingWindow.front().SLAM_msg = SLAM_msg;
    //part<1> Rotation.
    cout<<"addBlockSLAM() : part 1."<<endl;
    auto pEdgeSlamRotation = new EdgeAttitude();
    //shared_ptr<g2o::OptimizableGraph::Edge *> ptr_slam(pEdgeSlam);
    //pEdgeSlam->setId(this->EdgeVec.size());//TODO
    //this->EdgeVec.push_back(ptr_slam);
    int newest_vpr_id = this->newest_frame_id + 0;
    
    if(this->optimizer.vertex(newest_vpr_id) == NULL)
    {
        cout<<"error:(in addBlockSLAM() ) optimizer.vertex(0) is NULL!"<<endl;
    }
    pEdgeSlamRotation->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(newest_vpr_id)));
    auto q = SLAM_msg.pose.orientation;
    
    auto p = SLAM_msg.pose.position;
    
    
    
    cout<<"addBlockSLAM():setting quaternion"<<endl;
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    auto R_ = q_.toRotationMatrix();
    R_ = R_* this->SLAM_to_UAV_coordinate_transfer_R;
    Vector3d t_(p.x,p.y,p.z);
    cout<<"TODO:fix +t!!!"<<endl;
    t_ = this->SLAM_to_UAV_coordinate_transfer_R*t_ + this->SLAM_to_UAV_coordinate_transfer_t;cout<<"[SLAM_INFO]Using independent coordinate."<<endl;
    
    //t_ = this->SLAM_to_UAV_coordinate_transfer_R*(Vector3d(p.x,p.y,p.z) - this->last_position) + this->SLAM_to_UAV_coordinate_transfer_t;cout<<"[SLAM_INFO]Using Intergrating coordinate"<<endl;
    cout<<"[SLAM_INFO] slam_original_pose: "<<p.x<<","<<p.y<<","<<p.z<<endl;
    cout<<"[SLAM_INFO] SLAM_to_UAV_coordinate_transfer_R:\n"<<SLAM_to_UAV_coordinate_transfer_R<<endl;
    cout <<"[SLAM_INFO] slam position:"<<t_[0]<<","<<t_[1]<<","<<t_[2]<<endl;
    
    
    
    
   
    
    Eigen::Matrix<double,3,3> info_mat_slam_rotation = Eigen::Matrix<double,3,3>::Identity();
    cout<<"addBlockSLAM():part 3"<<endl;
    
    //VertexPR slam_pr_R_only;
    
    //slam_pr_R_only.R() = q_.toRotationMatrix().topLeftCorner(3,3);
    auto se3_slam = SO3d::log(q_.toRotationMatrix() );
    pEdgeSlamRotation->setMeasurement(se3_slam);
    pEdgeSlamRotation->setInformation(info_mat_slam_rotation);
    cout <<"Measurement,information mat set.Adding edge slam!!!"<<endl;
    const bool SLAM_ROTATION_USE_ROBUST_KERNEL = true;
    if(SLAM_ROTATION_USE_ROBUST_KERNEL)
    {
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        pEdgeSlamRotation->setRobustKernel(rk);
        rk->setDelta(thHuber);
    }
    this->optimizer.addEdge(pEdgeSlamRotation);
    cout<<"Edge SLAM added successfully."<<endl;
    //part<2> Translation
    
    
    //this->autoInsertSpeedVertexAfterInsertCurrentVertexPR();
    auto pEdgeSlamTranslation = new EdgePRGPS();
    auto slam_t_measurement = t_;
    pEdgeSlamTranslation->setMeasurement(slam_t_measurement);
    Eigen::Matrix<double,3,3> t_info_mat = Eigen::Matrix<double,3,3>::Identity();
    double slam_t_info_weight = (this->fSettings)["SLAM_T_INFO_WEIGHT"];
    pEdgeSlamTranslation->setInformation(t_info_mat*slam_t_info_weight);
    pEdgeSlamTranslation->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(newest_vpr_id)));
    const bool SLAM_TRANSLATION_USE_ROBUST_KERNEL = true;
    if(SLAM_TRANSLATION_USE_ROBUST_KERNEL)
    {
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        pEdgeSlamTranslation->setRobustKernel(rk);
        rk->setDelta(thHuber);
    }
    this->optimizer.addEdge(pEdgeSlamTranslation);
    //Edge PRV.
    /*//TODO:fill Edge SLAM PRV.
    if(this->historyStatesWindow.size()>0)
    {
        shared_ptr<Edge_SLAM_PRV>pEdge_SLAM_PRV(new Edge_SLAM_PRV());
        pEdge_SLAM_PRV->setId(this->EdgeVec.size());
        this->EdgeVec.push_back(pEdge_SLAM_PRV);
        //TODO:fit in type.
        pEdge_SLAM_PRV->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>&(this->historyStatesWindow[this->historyStatesWindow.size()-1]));//old one.
        pEdge_SLAM_PRV->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));
        pEdge_SLAM_PRV->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex *> (&(this->historySpeed[this->historySpeed.size()-1])));
        pEdge_SLAM_PRV->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex *> (&this->currentSpeed));
        g2o::OptimizableGraph::Vertex::
        //pEdge_SLAM_PRV->setInformation(....);//TODO
    }
    else
    {//just add translation.
        shared_ptr<EdgePRGPS> pEdgePositionSLAM(new EdgePRGPS());
        pEdgePositionSLAM->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get());
        //calc infomation mat from multiple/single slam msg.May be it can be estimated from points num or quality.
        //pEdgePositionSLAM->setInformation(...);TODO
    }*/
    //step<2> add edge_slam_prv
    if(this->SlidingWindow.size()<3)//只有一帧，放弃。
    {
        cout<<"Edge count <3.return."<<endl;
        return;
    }
    GOG_Frame* lastFrame = &(this->SlidingWindow[1]);
    GOG_Frame* thisFrame = &(this->SlidingWindow.front());
    cout<<"Frame queryed."<<endl;
    double dt = thisFrame->slam_frame_time - lastFrame->slam_frame_time;
    auto pEdgeSLAMPRV = new EdgeSLAMPRV(Vector3d(0,0,0));//4 vertices:pr_i,pr_j,speed_i,speed_j
    int vpr_j_id = this->newest_frame_id;
    int vspeed_j_id = vpr_j_id+1;
    int vpr_i_id = vpr_j_id-2;
    int vspeed_i_id = vpr_i_id+1;
    
    //check memory.
    bool vertex_memory_correct = true;
    for (int i=0;i<4;i++)
    {
        if(this->optimizer.vertex(vpr_i_id+i) == NULL)
        {
            vertex_memory_correct = false;
            cout<<"Error in EdgeSLAMPRV:memory not correct!i:"<<i<<endl;
        }
    }
    if(!vertex_memory_correct)
    {
        return;
    }
    
    pEdgeSLAMPRV->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(vpr_i_id)) );
    pEdgeSLAMPRV->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(vpr_j_id)) );
    pEdgeSLAMPRV->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(vspeed_i_id)) );
    pEdgeSLAMPRV->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->optimizer.vertex(vspeed_j_id)) );
    cout<<"SLAMEdgePRV Vertices set."<<endl;
    auto slam_preint = new IMUPreIntegration;
    
//IMU preint 这个对象更新的方法如下：
    
    //Vector3d bg = mpLastKeyFrame->BiasG();//设置IMU bias，对于我们的SLAM测速不需要。
    //Vector3d ba = mpLastKeyFrame->BiasA();

    //const IMUData &imu = mvIMUSinceLastKF.front();//读取最早的imu消息时间。
    //dt 已经计算.
    //没有什么可update的，这块略过。
    auto last_position = lastFrame->SLAM_msg.pose.position;
    Vector3d p_last(last_position.x,last_position.y,last_position.z);
    
    slam_preint->_delta_P = t_ - (this->SLAM_to_UAV_coordinate_transfer_R* - p_last);
    cout <<"DeltaP set."<<endl;
    //if(this->SlidingWindow.size()<3)
    if(1) //TODO:DEBUG ONLY!
    {
        slam_preint->_delta_V = Vector3d(0,0,0);//set speed estimation to 0;
    }
    //slam_preint->deltaV = 
    cout<<"DeltaV set."<<endl;
    auto last_q = SLAM_msg.pose.orientation;
    Eigen::Quaterniond last_q_;
    last_q_.w() = last_q.w;
    last_q_.x() = last_q.x;
    last_q_.y() = last_q.y;
    last_q_.z() = last_q.z;
    
    auto last_R = last_q_.toRotationMatrix();
    slam_preint->_delta_R = q_.toRotationMatrix() * last_R.inverse();
    cout<<"DeltaR set."<<endl;
    /*
    IMUPreInt.update(imu.mfGyro - bg, imu.mfAcce - ba, dt); //更新。

    // integrate each imu
    for (size_t i = 0; i < mvIMUSinceLastKF.size(); i++) {
        const IMUData &imu = mvIMUSinceLastKF[i];
        double nextt;
    
        // 
        if (i == mvIMUSinceLastKF.size() - 1) 
            nextt = mpCurrentFrame->mTimeStamp;         // last IMU, next is this KeyFrame
        else
            nextt = mvIMUSinceLastKF[i + 1].mfTimeStamp;  // regular condition, next is imu data
        // delta time
        double dt = nextt - imu.mfTimeStamp;
        // update pre-integrator
        IMUPreInt.update(imu.mfGyro - bg, imu.mfAcce - ba, dt); 
    }*/

    pEdgeSLAMPRV->setMeasurement(*slam_preint);
    cout<<"Measurement set."<<endl;
    pEdgeSLAMPRV->setInformation(Matrix9d::Identity());
    cout<<"Information set."<<endl;
    pEdgeSLAMPRV->setLevel(0);
    cout<<"Level set."<<endl;
    const bool SLAM_EPRV_USE_ROBUST_KERNEL = true;
    if(SLAM_EPRV_USE_ROBUST_KERNEL)
    {
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        pEdgeSLAMPRV->setRobustKernel(rk);
        rk->setDelta(5);//小一些的delta,容易忽略极端值。
    }
    this->optimizer.addEdge(pEdgeSLAMPRV);
    cout<<"Edge PRV added to optimization graph."<<endl;
    
    //pEdgeSLAMPRV.setInformation();
/*      Matrix9d CovPRV = imupreint.getCovPVPhi();
        // 但是Edge里用是P,R,V，所以交换顺序
        //这里要考虑到删掉了一些行列。
        CovPRV.col(3).swap(CovPRV.col(6));
        CovPRV.col(4).swap(CovPRV.col(7));
        CovPRV.col(5).swap(CovPRV.col(8));
        CovPRV.row(3).swap(CovPRV.row(6));
        CovPRV.row(4).swap(CovPRV.row(7));
        CovPRV.row(5).swap(CovPRV.row(8));

        // information matrix
        ePRV->setInformation(CovPRV.inverse());
*/
    
    

    //TODO
}
void GlobalOptimizationGraph::addSLAM_edgeprv(const geometry_msgs::PoseStamped& SLAM_msg)
{
    //step<1>.Form a IMU_Preintegration like object so we can form edgeprv.
    //TODO:set DV.
    ygz::IMUPreIntegration preintv;
    //preintv.setDP(SLAM_msg.pose-init_pose....);
    //preintv.setDV();
    /*
    ygz::EdgeSLAMPRV * pPRV = new EdgeSLAMPRV();
    pPRV->setVertex(0,....);//vertex pr1
    pPRV->setVertex(1,....);//vertex pr2
    pPRV->setVertex(...)//vertex speed1
    pPRV->setVertex(...)//vertex speed2
    */
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




