#ifndef __BSPLINE_OPTIMIZER_H__
#define __BSPLINE_OPTIMIZER_H__

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include<IpTNLP.hpp>
#include "path_optimization/cubic_bspline.h"

class BsplineOptimizer
{
private:
  NonUniformBspline* bspline_;
  
  double lambda1_;
  double lambda2_;
  double lambda3_;
  double lambda4_;
  double lambda5_;
  
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  
public:
  BsplineOptimizer();
  void setBspline(NonUniformBspline *bspline);
  void optimize();
  void setParam();
  
  
};

#endif // __BSPLINE_OPTIMIZER_H__
