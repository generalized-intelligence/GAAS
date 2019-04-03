
#ifndef GLOBAL_OPTIMIZATION_GRAPH_H
#define GLOBAL_OPTIMIZATION_GRAPH_H


#include "G2OTypes.h"
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
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


//struct State//the prev-state of drone.
//{
//    
//};
typedef VertexPR State;
typedef VertexPR Speed;
const int GPS_INIT_BUFFER_SIZE = 100;

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
    //TODO:Init a GPS callback buffer block class,inherit callback buffer base,implement init and check avail.

    bool inputGPS(const sensor_msgs::NavSatFix& gps);
    bool tryInitVelocity();
    inline bool isWorkingWithGPS()
    {
        return(this->allow_gps_usage&& this->gps_init_success);
    }
    void addBlockAHRS();
    void addBlockSLAM();
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();
    void addGPS();
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
    RotationMatrix ahrs_R;

    //optimization graph.
    G2OOptimizationGraph graph;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg *solver;
    int EdgeID = 0;
    int VertexID = 0;
    //gps configuration and initiation.
    void remap_UAV_coordinate_with_GPS_coordinate();
    Coordinate GPS_coord;

    bool allow_gps_usage = true;//can be configured by config.yaml
    bool gps_init_success = false;
    double gps_init_longitude,gps_init_latitude,gps_init_altitude;
    std::vector<sensor_msgs::NavSatFix> gps_info_buffer;
    //scene part.
    void fix_scene_Rotation_with_AHRS();
    void remap_scene_coordinate_with_GPS_coordinate();
    void remap_scene_coordinate_with_UAV_coordinate();
    VertexPR relative_scene_to_UAV_body;
};




#endif




