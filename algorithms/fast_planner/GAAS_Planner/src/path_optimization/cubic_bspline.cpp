#include "path_optimization/cubic_bspline.h"

CubicBspline::CubicBspline()
{
  pb_ = 3;
  setParam();
}

CubicBspline::CubicBspline(Eigen::MatrixXd control_points, double dt, int pb)
{
  pb_ = pb;
  //k_ = pb_ + 1;
  control_pts_ = control_points;
  dt_ = dt;
  
  n_ = control_points.rows()-1;
  m_ = n_ + pb_ + 1;
  
  knots_ = Eigen::VectorXd::Zero(m_+1);
  
  //valid knots span: t ∈ knots_[pb_:m_-pb_]
  for (int i = 0; i <= m_; ++i)
  {
    if (i <= pb_)
      knots_(i) = double(-pb_+i) * dt_;
    else
      knots_(i) = knots_(i-1) + dt_;
  }
  setParam();
}

void CubicBspline::setParam()
{
  max_vel_ = 2;
  max_acc_ = 1;
}


Eigen::MatrixXd CubicBspline::getDerivativeControlPoints()
{
  Eigen::MatrixXd new_ctp = Eigen::MatrixXd::Zero(control_pts_.rows()-1, 3);
  for (int i = 0; i < new_ctp.rows(); ++i)
  {
    new_ctp.row(i) = pb_ * (control_pts_.row(i+1) - control_pts_.row(i))/(knots_(i+pb_+1) - knots_(i+1));
  }
  return new_ctp;
}

CubicBspline CubicBspline::getDerivative()
{
  Eigen::MatrixXd new_ctp = this->getDerivativeControlPoints();
  CubicBspline derivative = CubicBspline(new_ctp, dt_, pb_-1);
  Eigen::VectorXd new_knots(knots_.rows()-2);
  new_knots = knots_.segment(1, knots_.rows() - 2);
  derivative.setKnot(new_knots);
  return derivative;
}

bool CubicBspline::checkFeasibility()
{
  //Check vel.
  //Vi = (pb_*(Qi+1-Qi)) / (ti+pb_+1 - ti+1)
  
}

Eigen::MatrixXd CubicBspline::getControlPoints()
{
  return control_pts_;
}

void CubicBspline::setKnot(Eigen::VectorXd knots)
{
  knots_ = knots;
}

Eigen::VectorXd CubicBspline::getKnot()
{
  return knots_;
}

void CubicBspline::getValidTimeSpan(double& um, double& um_p)
{
  //valid knots span: t ∈ knots_[pb_:m_-pb_]
  um = knots_(pb_);
  um_p = knots_(m_-pb_);
}

Eigen::Vector3d CubicBspline::deBoorCox(double t)
{
  if (t < knots_(pb_) || t > knots_(m_ - pb_))
  {
    if (t < knots_(pb_))
      t = knots_(pb_);
    if (t > knots_(m_-pb_))
      t = knots_(m_-pb_);
  }

  // find out which [ui,ui+1] lay in
  int k = pb_;
  while (knots_(k+1)<t)
  {
    k++;
  }
  
  // B(t) = ∑PiB(t) t ∈ knots_[pb_:m_-pb_]
  std::vector<Eigen::Vector3d> P;
  for (int i=0; i <= pb_; i++)
  {
    P.push_back(control_pts_.row(k-pb_+i));
  }

  for (int r=1; r <= pb_; r++)
  {
    for (int i = pb_; i >= r; i--)
    {
      double alpha = (t - knots_[i + k - pb_]) / (knots_[i + 1 + k - r] - knots_[i + k - pb_]);
      P[i] = (1-alpha) * P[i-1] + alpha * P[i];
    }
  }

  return P[pb_];
}

bool CubicBspline::reallocateTime()
{

}

void CubicBspline::setControlPointFromValuePoint(Eigen::MatrixXd sample_pts, double dt)
{
  //Position value point:
  //p(s(t)) = s(t).T * M * qm
  //s(t) = [1, s(t), s(t)^2, s(t)^3].T
  //qm = [Qm-pb, Qm-pb+1, Qm-pb+2, Qm]
  //pb = 3
  //A * X = sample_pts => solve X; X = control_points
  //T=s(t); s(t)=t/dt; dt = tm+1 - tm
  //P(s(t)) = ((-P0+3P1-3P2+P3)T^3)/6 + ((P0-2P1+P2)T^2)/2 + 
  //		  ((-P0+P2)T)/2 + (P0 + 4P1 + P2)/6
  
  int K = sample_pts.cols() - 5;
  
  Eigen::VectorXd prow(3), vrow(3), arow(3);
  prow << 1/6.0 * 1, 1/6.0 * 4, 1/6.0 * 1;			//1/6 * (P0 + 4P1 + P2)
  vrow << (1/2.0/dt) * (-1) , 0,  (1/2.0/dt) * 1;		//1/2/dt * (-P0 + P2)
  arow << (1/dt/dt) * 1, (1/dt/dt)*(-2), (1/dt/dt)* 1;		//1/dt/dt * (P0 - 2P1 + P2)
  

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K+5, K+4);

  for (int i = 0; i < K+2; ++i)
    A.block(i, i, 1, 3) = prow.transpose();

  A.block(K+2, 0, 1, 3) = A.block(K+3, K+1, 1, 3) = vrow.transpose();
  A.block(K+4, 0, 1, 3) = arow.transpose();


  Eigen::VectorXd bx(K+5), by(K+5), bz(K+5);
  for (int i = 0; i < K+5; ++i)
  {
    bx(i) = sample_pts(0, i);
    by(i) = sample_pts(1, i);
    bz(i) = sample_pts(2, i);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  control_pts_.resize(K+4, 3);
  control_pts_.col(0) = px;
  control_pts_.col(1) = py;
  control_pts_.col(2) = pz;
  

//   int K = sample_pts.cols() - 4 - 1;
//   
//   Eigen::MatrixXd control_pts;
// 
//   // write A
//   Eigen::VectorXd prow(3), vrow(3), arow(3);
//   prow << 1, 4, 1;
//   vrow << -1, 0, 1;
//   arow << 1, -2, 1;
// 
//   Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 5, K + 4);
// 
//   for (int i = 0; i < K + 2; ++i)
//     A.block(i, i, 1, 3) = prow.transpose();
// 
//   A.block(K + 2, 0, 1, 3) = A.block(K + 3, K + 1, 1, 3) = vrow.transpose();
//   A.block(K + 4, 0, 1, 3) = arow.transpose();
// 
//   // cout << "A:\n" << A << endl;
//   A.block(0, 0, K + 2, K + 4) = (1 / 6.0) * A.block(0, 0, K + 2, K + 4);
//   A.block(K + 2, 0, 2, K + 4) = (1 / 2.0 / dt) * A.block(K + 2, 0, 2, K + 4);
//   A.row(K + 4) = (1 / dt / dt) * A.row(K + 4);
// 
//   // write b
//   Eigen::VectorXd bx(K + 5), by(K + 5), bz(K + 5);
//   for (int i = 0; i < K + 5; ++i)
//   {
//     bx(i) = sample_pts(0, i);
//     by(i) = sample_pts(1, i);
//     bz(i) = sample_pts(2, i);
//   }
// 
//   // solve Ax = b
//   Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
//   Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
//   Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);
// 
//   // convert to control pts
//   control_pts.resize(K + 4, 3);
//   control_pts.col(0) = px;
//   control_pts.col(1) = py;
//   control_pts.col(2) = pz;
//   
//   control_pts_ = control_pts;
  
  pb_ = 3;
  dt_ = dt;

  n_ = control_pts_.rows()-1;
  m_ = n_ + pb_ + 1;
  knots_ = Eigen::VectorXd::Zero(m_+1);

  //valid knots span: t ∈ knots_[pb_:m_-pb_]
  for (int i = 0; i <= m_; ++i)
  {
    if (i <= pb_)
      knots_(i) = double(-pb_+i) * dt_;
    else
      knots_(i) = knots_(i-1) + dt_;
  }

  setParam();
  
}



