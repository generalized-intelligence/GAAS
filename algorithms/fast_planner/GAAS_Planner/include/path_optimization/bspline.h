#ifndef __BSPLINE_H__
#define __BSPLINE_H__

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

class NonUniformBspline
{
private:
  Eigen::MatrixXd control_points_;
  Eigen::VectorXd knots_;
  double t_span_;
  
  int order_;
  
  Eigen::Vector3d p0_, v0_, a0_;
  
public:
  NonUniformBspline();
  NonUniformBspline(Eigen::MatrixXd points, int order, double interval);
  
  void setKnot(Eigen::VectorXd knot);
  Eigen::VectorXd getKnot();
  void getTimeSpan(double& um, double& um_p);
  
  void getControlPointEqu3(Eigen::MatrixXd samples, double ts, Eigen::MatrixXd& control_pts);
  
};

#endif // __BSPLINE_H__
