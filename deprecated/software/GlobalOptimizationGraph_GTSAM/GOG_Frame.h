#ifndef GOG_FRAME_H
#define GOG_FRAME_H
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
