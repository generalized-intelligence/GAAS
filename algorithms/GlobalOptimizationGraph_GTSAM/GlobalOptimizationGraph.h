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
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

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

#include <ctime>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

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
    void resetOptimizationGraph(){};

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

    inline cv::Mat PoseStampedToMat(const geometry_msgs::PoseStamped& pose)
    {
        Eigen::Quaterniond pose_q(pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z);

        Eigen::Matrix3d pose_R = pose_q.toRotationMatrix();

        cv::Mat mat_R;
        cv::eigen2cv(pose_R, mat_R);

        cv::Mat T = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);

        T.at<double>(0, 0) = mat_R.at<double>(0, 0);
        T.at<double>(0, 1) = mat_R.at<double>(0, 1);
        T.at<double>(0, 2) = mat_R.at<double>(0, 2);
        T.at<double>(1, 0) = mat_R.at<double>(1, 0);
        T.at<double>(1, 1) = mat_R.at<double>(1, 1);
        T.at<double>(1, 2) = mat_R.at<double>(1, 2);
        T.at<double>(2, 0) = mat_R.at<double>(2, 0);
        T.at<double>(2, 1) = mat_R.at<double>(2, 1);
        T.at<double>(2, 2) = mat_R.at<double>(2, 2);

        T.at<double>(0, 3) = pose.pose.position.x;
        T.at<double>(1, 3) = pose.pose.position.y;
        T.at<double>(2, 3) = pose.pose.position.z;
        T.at<double>(3, 3) = 1;

        return T;
    }

    inline cv::Mat findRelativeTransformMat(const geometry_msgs::PoseStamped& Pwb1, const geometry_msgs::PoseStamped& Pwb2)
    {
        auto Twb1 = PoseStampedToMat(Pwb1);
        auto Twb2 = PoseStampedToMat(Pwb2);

        // try to find Tb1b2
        // Tb1b2 = Tb1w * Twb2;
        auto Tb1b2 = Twb1.inv() * Twb2;
        LOG(INFO)<<"findRelativeTransformMat Twb1: \n"<<Twb1<<endl;
        LOG(INFO)<<"findRelativeTransformMat Twb2: \n"<<Twb2<<endl;
        LOG(INFO)<<"findRelativeTransformMat Tb1b2: \n"<<Tb1b2<<endl;
        return Tb1b2;
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
    //IncrementalFixedLagSmoother* p_fixed_lag_smoother;
    ISAM2Params isam2_params_;

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

    double gps_init_longitude, gps_init_latitude, gps_init_altitude;
    double gps_init_lon_variance, gps_init_lat_variance, gps_init_alt_variance;

    std::vector<sensor_msgs::NavSatFix> gps_info_buffer;
    //time for calc speed vertex
    double last_slam_msg_time;
    double init_slam_msg_time;

    OptimizationGraphStatusT current_status;
    std::mutex state_mutex;

    ofstream mInputSlamFile;
    ofstream mGPSPathFile;

    std::chrono::time_point<std::chrono::system_clock> gps_time_last, gps_time_current, slam_time_current;
    std::chrono::duration<double> elapsed_seconds_since_last_start;
    vector<std::chrono::duration<double>> elapsed_time_vec;

    bool gps_lost = false;
};



#endif
