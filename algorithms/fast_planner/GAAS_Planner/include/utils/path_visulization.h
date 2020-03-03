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

class path_visulization
{
public:
  void drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id);
  void displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color,
                                              int id);
};

#endif // __PATH_VISULIZATION_H_
