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
using namespace std;
using namespace ygz;
class GOG_Frame
{
public:
    GOG_Frame()
    {;}
    //VertexPR* pPRVertex;
    //VertexSpeed* pSpeedVertex;
    double slam_frame_time;
    geometry_msgs::PoseStamped SLAM_msg;
};
#endif
