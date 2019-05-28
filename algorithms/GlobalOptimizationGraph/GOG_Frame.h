#ifndef GOG_FRAME_H
#define GOG_FRAME_H
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimizable_graph.h>
#include "G2OTypes.h"
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
#include "utils.h"
using namespace std;
using namespace ygz;
class GOG_Frame
{
public:
    GOG_Frame()
    {;}
    VertexPR* pPRVertex;
    VertexSpeed* pSpeedVertex;
    //double slam_frame_time;
    //long long int slam_frame_time;
    time_us_t slam_frame_time;//有些包时间戳有问题，时间差无穷小，速度将是无穷大，用接收时间代替真实时间确保数值稳定。
    geometry_msgs::PoseStamped SLAM_msg;
};
#endif
