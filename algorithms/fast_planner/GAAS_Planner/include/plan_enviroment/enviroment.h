#ifndef __ENVIROMENT_H__
#define __ENVIROMENT_H__

#include <voxblox_ros/esdf_server.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>

class SdfEnviroment
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  
  voxblox::EsdfServer voxblox_server_;

public:
  SdfEnviroment(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  
  double getDistance(const Eigen::Vector3d& position) ;
  
  double getDistanceAndGradiend(const Eigen::Vector3d& position, Eigen::Vector3d &gradient) ;
  
  typedef std::shared_ptr<SdfEnviroment> Ptr;
};

#endif // __ENVIROMENT_H__
