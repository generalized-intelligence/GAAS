#include "plan_enviroment/enviroment.h"

SdfEnviroment::SdfEnviroment(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
							voxblox_server_(nh,nh_private)
{
}

double SdfEnviroment::getDistance(const Eigen::Vector3d& position) 
{
  double distance = 0;
  
  if (!voxblox_server_.getEsdfMapPtr()) 
  {
    LOG(ERROR)<<"[ENV]getDistance: No map.";
    return -1.0;
  }
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance))
  {
    //Can't find pos at esdf map, so return a big distance.
    DLOG(ERROR)<<"no distance.";
    return 1000;
  }
  return distance;
}

double SdfEnviroment::getDistanceAndGradiend(const Eigen::Vector3d& position, Eigen::Vector3d &gradient) 
{
  if (!voxblox_server_.getEsdfMapPtr()) 
  {
    LOG(ERROR)<<"[ENV]getDistance: No map.";
    return -1.0;
  }
  double distance = -2.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(position,
                                                              &distance, &gradient)) 
  {
    //Can't find pos at esdf map, so return a big distance.
    Eigen::Vector3d zero(0,0,0);
    gradient = zero;
    return 1000;
    //return -2.0;
  }
  return distance;
}



