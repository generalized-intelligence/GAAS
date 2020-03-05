#ifndef __PATH_VISULIZATION_H_
#define __PATH_VISULIZATION_H_


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>
#include <glog/logging.h>
#include <vector>

class PathVisulization
{
private:
  ros::NodeHandle nh_;
  ros::Publisher traj_pub;
public:
  PathVisulization(ros::NodeHandle& nh);
  void drawPath(std::vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color);
  void displaySphereList(std::vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color);
};

#endif // __PATH_VISULIZATION_H_
