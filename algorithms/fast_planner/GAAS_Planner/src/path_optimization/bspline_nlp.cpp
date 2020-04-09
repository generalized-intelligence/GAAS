#include "path_optimization/bspline_nlp.h"

BsplineTNLP::BsplineTNLP(const int N, Eigen::MatrixXd& control_pts, double dt, SdfEnviroment& sdf_map)
			: N_(N), control_pts_(control_pts), dt_(dt), sdf_map_(&sdf_map)
{
  setParam();
}

BsplineTNLP::BsplineTNLP(const int N, Eigen::MatrixXd& control_pts, double dt)
			: N_(N), control_pts_(control_pts), dt_(dt)
{
  
  DLOG(INFO) << "It have "<<N<<" variable to optimize. Now please set Enviroment";
}



BsplineTNLP::~BsplineTNLP()
{
  
}

void BsplineTNLP::setParam()
{
  vmax_ = 2;
  amax_ = 1;
}


bool BsplineTNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style)
{
  n = N_;
  m = 0;
  nnz_h_lag = 0;
  nnz_jac_g = 0;
  index_style = C_STYLE;
  return true;
}


bool BsplineTNLP::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)
{
  LOG(INFO)<<"Start BOUND.";
  assert(n==N_);
  
  for(Index i=0; i<n; i++)
  {
    x_l[i] = -1e9;
    x_u[i] = 1e9;
  }
  LOG(INFO)<<"END BOUND.";
  return true;
}

bool BsplineTNLP::get_starting_point(Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda, Number* lambda)
{
  assert(n==N_);
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  
  LOG(INFO)<<"Start point.";
  //x = q_;
  //order = 3;
  //TODO:Add end point optimize.
  
  for (int i=0; i<control_pts_.rows(); i++)
  {
    if(i<3)
      continue;
    
    if(i>=(control_pts_.rows()-3))
      continue;
    
    for(int j=0; j<3;j++)
      x[3 * (i-3) + j] = control_pts_(i, j);
  }
  
  
  
  LOG(INFO)<<"END Start point.";
  
  return true;
}

bool BsplineTNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  //LOG(INFO)<<"Start eval";
  //TODO:Add end point optimize.
//   std::vector<Eigen::Vector3d> q;
//   for (int i = 0; i < 3; i++)
//     q.push_back(control_pts_.row(i));
//   
//   for (int i = 0; i < N_ / 3; i++)
//   {
//     Eigen::Vector3d qi(x[3 *i], x[3*i + 1], x[3*i + 2]);
//     q.push_back(qi);
//   }
//   
//   for (int i = 0; i < 3; i++)
//     q.push_back(control_pts_.row(control_pts_.rows()-3 + i));
//   
//   LOG(INFO)<<"Start q";
//   double c_smoothness = getSmoothnessCost(q);
//   LOG(INFO)<<"Start q1: " << c_smoothness;
//   double c_distance = getDistanceCost(q);
//   LOG(INFO)<<"Start q2: "<<c_distance;
//   double c_feasible = getFeasibilityCost(q);
//   LOG(INFO)<<"Start q3: "<<c_feasible;
//   double c_endpoint = getEndpointCost(q);
//   LOG(INFO)<<"Start q4: "<<c_endpoint;
//   
  
  std::vector<Eigen::Vector3d> q;
  for (int i = 0; i < 3; i++)
    q.push_back(control_pts_.row(i));
  
  for (int i = 0; i < N_ / 3; i++)
  {
    Eigen::Vector3d qi(x[3 *i], x[3*i + 1], x[3*i + 2]);
    q.push_back(qi);
  }
  
  for (int i = 0; i < 3; i++)
    q.push_back(control_pts_.row(control_pts_.rows()-3 + i));
  
  //LOG(INFO)<<"Start q";
  tmp_c_smoothness_ = getSmoothnessCost(q);
  tmp_c_distance_ = getDistanceCost(q);
  tmp_c_feasible_ = getFeasibilityCost(q);
  tmp_c_endpoint_ = getEndpointCost(q);
  //obj_value = tmp_c_smoothness_;// + 0.00001*tmp_c_feasible_ + 0.001*tmp_c_endpoint_;
  
  obj_value = 10.0 * tmp_c_smoothness_ + 0.6*tmp_c_distance_ + 0.00001*tmp_c_feasible_ + 0.001*tmp_c_endpoint_;
 // LOG(INFO)<<"Start q46: "<<obj_value;
  return true;
}

bool BsplineTNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  //LOG(INFO)<<"Start grad.";
  
  std::vector<Eigen::Vector3d> q;
  for (int i = 0; i < 3; i++)
    q.push_back(control_pts_.row(i));
  
  for (int i = 0; i < N_ / 3; i++)
  {
    Eigen::Vector3d qi(x[3 *i], x[3*i + 1], x[3*i + 2]);
    q.push_back(qi);
  }
  
  for (int i = 0; i < 3; i++)
    q.push_back(control_pts_.row(control_pts_.rows()-3 + i));
  
  //LOG(INFO)<<"Start q";
  tmp_c_smoothness_ = getSmoothnessCost(q);
  //LOG(INFO)<<"Start q1: " << tmp_c_smoothness_;
  tmp_c_distance_ = getDistanceCost(q);
  //LOG(INFO)<<"Start q2: "<<tmp_c_distance_;
  tmp_c_feasible_ = getFeasibilityCost(q);
  //LOG(INFO)<<"Start q3: "<<tmp_c_feasible_;
  tmp_c_endpoint_ = getEndpointCost(q);
  //LOG(INFO)<<"Start q4: "<<tmp_c_endpoint_;
  
  
  for(int i=0; i<N_/3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      //grad_f[3*i+j] = tmp_g_smoothness_[i+3](j);// + 
			//0.00001*tmp_g_feasible_[i+3](j) + 0.001*tmp_g_endpoint_[i+3](j);
     grad_f[3*i+j] = 10.0*tmp_g_smoothness_[i+3](j) + 0.6*tmp_g_distance_[i+3](j) + 
			0.00001*tmp_g_feasible_[i+3](j) + 0.001*tmp_g_endpoint_[i+3](j);
    }
  }
  //LOG(INFO)<<"end grad.";
  return true;
}

bool BsplineTNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  return false;
}

bool BsplineTNLP::eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
  return false;
}

bool BsplineTNLP::eval_h(Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda, bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values)
{
  return false;
}

void BsplineTNLP::finalize_solution(SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U, Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
  LOG(INFO)<<"END IPOPT :" << N_;
  best_q_ = std::vector<double>(&x[0], &x[0]+N_);
}

double BsplineTNLP::getSmoothnessCost(std::vector<Eigen::Vector3d> &q)
{
  tmp_g_smoothness_.resize(control_pts_.rows());
  std::fill(tmp_g_smoothness_.begin(), tmp_g_smoothness_.end(), Eigen::Vector3d(0, 0, 0));
  Eigen::Vector3d jerk;
  double cost = 0;

  for (int i = 0; i < q.size() - 3; i++)
  {
    //P(s(t)) = ((-P0+3P1-3P2+P3)T^3)/6 + ((P0-2P1+P2)T^2)/2 + 
    //		  ((-P0+P2)T)/2 + (P0 + 4P1 + P2)/6

    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();

    tmp_g_smoothness_[i + 0] += 2.0 * jerk * (-1.0);
    tmp_g_smoothness_[i + 1] += 2.0 * jerk * (3.0);
    tmp_g_smoothness_[i + 2] += 2.0 * jerk * (-3.0);
    tmp_g_smoothness_[i + 3] += 2.0 * jerk * (1.0);
  }
  return cost;
}

double BsplineTNLP::getDistanceCost(std::vector<Eigen::Vector3d> &q)
{
  tmp_g_distance_.resize(control_pts_.rows());
  double cost = 0;
  std::fill(tmp_g_distance_.begin(), tmp_g_distance_.end(), Eigen::Vector3d(0, 0, 0));
  
  Eigen::Vector3d tmp_gradient;
  Eigen::Vector3d zero(0,0,0);
  
  for(int i=3; i<q.size()-3;i++)
  {
    double dist = sdf_map_->getDistanceAndGradiend(q[i], tmp_gradient);
    cost += (dist < 0.8) ? pow(dist-0.8, 2) : 0.0;
    tmp_g_distance_[i] += (dist < 0.8) ? 2.0 * (dist-0.8) * tmp_gradient : zero;
  }
  return cost;
  
}



double BsplineTNLP::getFeasibilityCost(std::vector<Eigen::Vector3d> &q)
{
  //According to the paper, feasibility cost is:
  // (v^2 - vmax^2)^2  if v^2 > vmax^2
  //  0		       if v^2 <= vmax^2
  
  tmp_g_feasible_.resize(control_pts_.rows());
  double cost = 0;
  std::fill(tmp_g_feasible_.begin(), tmp_g_feasible_.end(), Eigen::Vector3d(0, 0, 0));
  
  double vmax2 = vmax_*vmax_;
  double amax2 = amax_*amax_;
  
  for(int i=0; i<q.size()-1; i++)
  {
    Eigen::Vector3d vt = (q[i+1]-q[i]) / dt_;
    for (int j=0; j<3; j++)
    {
      double vd = (vt(j)*vt(j)) - vmax2;
      cost += vd > 0.0 ? pow(vd, 2) : 0.0;
      
      tmp_g_feasible_[i](j) += (vd > 0.0) ? ((-2)*vt(j)/dt_ * 2*vd) : 0.0;
      tmp_g_feasible_[i+1](j) += (vd > 0.0) ? ((2)*vt(j)/dt_ * 2*vd) : 0.0; 
    }
  }
  double dt2 = dt_*dt_;
  for(int i=0; i<q.size()-2; i++)
  {
    Eigen::Vector3d at = ((q[i+2]-q[i+1]) - (q[i+1]-q[i])) / dt2;
    for(int j=0; j<3; j++)
    {
      double ad = at(j) * at(j) - amax2;
      cost += (ad>0.0) ? pow(ad, 2) : 0.0;
      
      tmp_g_feasible_[i](j) += ad > 0.0 ? 2*at(j)/dt2 * 2*ad : 0.0;
      tmp_g_feasible_[i+1](j) += ad > 0.0 ? (-4)*at(j)/dt2 * 2*ad : 0.0;
      tmp_g_feasible_[i+2](j) += ad > 0.0 ? 2*at(j)/dt2 * 2*ad : 0.0;
    }
  }
  
  return cost;
}

double BsplineTNLP::getEndpointCost(std::vector<Eigen::Vector3d> &q)
{
  double cost = 0;
  tmp_g_endpoint_.resize(control_pts_.rows());
  std::fill(tmp_g_endpoint_.begin(), tmp_g_endpoint_.end(), Eigen::Vector3d(0, 0, 0));
  
  return cost;
}


