
#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>
#include "G2OTypes.h"
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "GPSExpand.h"
#include <cmath>
#include <opencv2/opencv.hpp>
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
typedef VertexPR State;
typedef VertexPR Speed;
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
    void addBlockAHRS(const nav_msgs::Odometry& AHRS_msg);
    void addBlockSLAM(const geometry_msgs::PoseStamped& SLAM_msg);
    void addBlockGPS(const sensor_msgs::NavSatFix& GPS_msg);
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();

    void doOptimization();
    void resetOptimizationGraph()
    {
        this->optimizer.clear();
    }
    bool estimateCurrentSpeed();

private:
    //status management.
    static const int STATUS_NO_GPS_NO_SCENE = 0; // a bit map.
    static const int STATUS_NO_GPS_WITH_SCENE = 1;
    static const int STATUS_WITH_GPS_NO_SCENE = 2;
    static const int STATUS_GPS_SCENE = 3;
    int GPS_AVAIL_MINIMUM;
    int status = STATUS_NO_GPS_NO_SCENE;
    void stateTransfer(int new_state);
    //uav location attitude info management.
    std::vector<State> historyStates;
    State currentState;

    Speed currentSpeed;
    std::vector<Speed> historySpeed;
    void resetSpeed();
    //SLAM
    VertexPR SLAM_to_UAV_coordinate_transfer;
    //AHRS
    Matrix3d ahrs_R_init;

    //optimization graph.
    //G2OOptimizationGraph graph;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg *solver;

    vector<shared_ptr<g2o::BaseVertex>> VertexVec;
    vector<shared_ptr<g2o::BaseEdge>> EdgeVec;

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
    VertexPR relative_scene_to_UAV_body;
};




#endif




