#ifndef __CUBIC_BSPLINE_H__
#define __CUBIC_BSPLINE_H__

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include <memory>
#include <glog/logging.h>

class CubicBspline
{
// Uniform Bspline
// Use Non-Uniform Bspline to enforce all control points of the first and
// second order derivatives within the feasible domain
  
private:
  Eigen::MatrixXd control_pts_;
  Eigen::VectorXd knots_;
  double dt_;
  
  int pb_, n_, m_;//, k_;
  
  double max_vel_, max_acc_;
  
  Eigen::MatrixXd getDerivativeControlPoints();
  
public:
  CubicBspline();
  CubicBspline(Eigen::MatrixXd control_pts, double dt, int pb=3);
  
  CubicBspline getDerivative();
  
  void setParam();
  
  Eigen::Vector3d deBoorCox(double t);
  
  Eigen::MatrixXd getControlPoints();
  
  void setKnot(Eigen::VectorXd knots);
  Eigen::VectorXd getKnot();
  void getValidTimeSpan(double& um, double& um_p);
  
  bool checkFeasibility();
  bool reallocateTime();
  
  void setControlPointFromValuePoint(Eigen::MatrixXd sample_pts, double dt);
  
  typedef std::shared_ptr<CubicBspline> Ptr;
  
};

#endif // __BSPLINE_H__
