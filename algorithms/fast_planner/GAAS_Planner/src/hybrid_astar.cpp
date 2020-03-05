#include "hybrid_astar.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

HybridAstar::HybridAstar()
{
  
}

HybridAstar::~HybridAstar()
{
  
}

void HybridAstar::reset()
{
  std::priority_queue<HybridNode*, std::vector<HybridNode*>, CompareHybrirdAstarNode> empty;
  open_set_.swap(empty);
  
  node_map_.clear();
  obstacle_map_.clear();
  path_node_.clear();
  body_.clear();
  movement_list_.clear();
}

void HybridAstar::setParam()
{
  //TODO:use cv config.
  resolution_ = 0.2;
  margin_ = 0.8; //use_resolution
  radius_ = 0.4;
  tie_ = 9.001;
  max_acc_ = 1;
  max_vel_ = 2;
  
  A_mat_ = Eigen::MatrixXd::Identity(6, 6);
  
  margin_g_ = std::ceil(margin_ / resolution_);
  radius_g_ = std::ceil(radius_/ resolution_);
  
  for(int i=-margin_g_; i<=margin_g_; i+=1)
    for(int j=-margin_g_; j<=margin_g_; j+=1)
      for(int k=-margin_g_; k<=margin_g_; k+=1)
      {
	body_.push_back(Eigen::Vector3d(i,j,k));
      }
}




void HybridAstar::getPathFromNode(HybridNode* end_node)
{
  HybridNode* current_obj = end_node;
  path_node_.push_back(end_node);
  while (current_obj->parent != NULL)
  {
    current_obj = current_obj->parent;
    path_node_.push_back(current_obj);
  }
 std::reverse(path_node_.begin(), path_node_.end());
}

void HybridAstar::setObstacle(std::vector< Eigen::Vector3d >& obstacle_list)
{
  std::cout<<"Obstacle Size: "<<obstacle_list.size()<<std::endl;
  for(int i=0;i<obstacle_list.size();i++)
  {
    std::cout<<"Obstacle : "<<obstacle_list[i](0)<<", "<<obstacle_list[i](1)
		<<", "<<obstacle_list[i](2)<<std::endl;
    
    auto it = obstacle_map_.find(posToGrid(obstacle_list[i]));
    if (!(it==obstacle_map_.end()))
    {
      continue;
    }
    obstacle_map_.insert(make_pair(posToGrid(obstacle_list[i]), obstacle_list[i]));
  }
  
}

int HybridAstar::findPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel)
{
  DLOG(INFO) << "Start search. Start pt: "<< start_pt;
  start_pt_ = start_pt;
  start_vel_ = start_vel;
  start_acc_ = start_acc;
  end_pt_ = end_pt;
  end_vel_ = end_vel;
  
  end_state_.head(3) = end_pt_;
  end_state_.tail(3) = end_vel_;
  
  if (!isValid(end_pt))
  {
    LOG(FATAL)<<"End point in Obstacle. End point: "<<end_pt(0)<<", "<<end_pt(1)<<", "<<end_pt(2);
    return 0;
  }
  
  HybridNode* current_node = new HybridNode();
  current_node->parent = nullptr;
  current_node->dynamic_state.head(3) = start_pt;
  current_node->dynamic_state.tail(3) = start_vel;
  current_node->grid_pos = posToGrid(start_pt);
  
  double time_to_goal;
  current_node->H = tie_* getHeuristic(current_node->dynamic_state, end_state_, time_to_goal);
  current_node->state = HybridNode::IN_OPEN_SET;
  
  open_set_.push(current_node);
  
  node_map_.insert(make_pair(current_node->grid_pos, current_node));
  
  while(!open_set_.empty())
  {
    current_node = open_set_.top();
    Eigen::Vector3d current_pos = current_node->dynamic_state.head(3);
    if((end_pt_-current_pos).norm() <= 0.8)
    {
      LOG(INFO) << "Success.";
      LOG(INFO) << current_pos;
      LOG(INFO) << end_pt_;
      LOG(INFO) << "dist: " <<(end_pt_-current_pos).norm();
      getPathFromNode(current_node);
      return 1;
    }
   // DLOG(INFO)<<current_node->dynamic_state(0) << ", " << current_node->dynamic_state(1)
	//<<", "<< current_node->dynamic_state(2);
    open_set_.pop();
    current_node->state = HybridNode::IN_CLOSE_SET;
    //close_set_.insert(current_node);
    extendRound(current_node);
    
  }
  return 0;
}

void HybridAstar::extendRound( HybridNode* current_obj)
{
  double T = 0.8;
  Eigen::Matrix<double, 6, 1> current_state = current_obj->dynamic_state;
  Eigen::Matrix<double, 6, 1> next_state;
  for (double ax = -max_acc_; ax <= max_acc_; ax+=0.5*max_acc_)
    for (double ay = -max_acc_; ay <= max_acc_; ay+=0.5*max_acc_)
      for (double az = -max_acc_; az <= max_acc_; az+=0.5*max_acc_)
      {
	Eigen::Vector3d input_a(ax, ay, 0.5*az);
	//std::cout << "Input: "<<ax<<", "<<ay << ", "<<az<<std::endl;
	stateTransit(current_state, next_state, input_a, T);
	//std::cout << "Next state: "<<next_state(0)<<", "<<next_state(1) << ", "<<next_state(2)<<" | "
	//    <<next_state(3)<<", "<<next_state(4) << ", "<<next_state(5)<<std::endl;
	
	Eigen::Vector3d next_pos = next_state.head(3);
	Eigen::Vector3d next_vel = next_state.tail(3);
	
	if(!(findInCloseset(next_pos)==nullptr))
	  continue;
	
	if(!isValid(next_pos))
	  continue;
	
	if(std::fabs(next_vel(0))>max_vel_ || std::fabs(next_vel(1))>max_vel_ 
		  || std::fabs(next_vel(2))>max_vel_)
	  continue;
	
	if ((current_obj->grid_pos - posToGrid(next_pos)).norm() == 0)
	  continue;
	
	Eigen::Matrix<double, 6, 1> ds;
	bool is_collision = false;
	for(int i=0; i<=5; i++)
	{
	  double dt = T * (i/5.0);
	  stateTransit(current_state, ds, input_a, dt);
	  Eigen::Vector3d dp = ds.head(3);
	  if(!isValid(dp))
	  {
	    //std::cout<<"!!!!!!!!!"<<dp(0)<<", "<<dp(1)<<", "<<dp(2)<<std::endl;
	    is_collision = true;
	    break;
	  }
	}
	
	if(is_collision==true)
	  continue;
	
	double new_g=0, new_h=0, time_to_end;
	new_g = (input_a.squaredNorm()+1)*T + current_obj->G;
	new_h = tie_*getHeuristic(next_state, end_state_, time_to_end);
	
	//std::cout<<"NEW G: " <<new_g<<" NEW_H: "<<new_h<<std::endl; 
	HybridNode* obj = findInOpenset(next_pos);
	if (!(obj == nullptr))
	{
	  if (new_g < obj->G)
	  {
	    obj->dynamic_state = next_state;
	    obj->G = new_g;
	    obj->input = input_a;
	    obj->duration = T;
	    obj->parent = current_obj;
	  }
	}
	else
	{
	  obj = new HybridNode();
	  obj->grid_pos = posToGrid(next_pos);
	  obj->G = new_g;
	  obj->H = new_h;
	  obj->input = input_a;
	  obj->duration = T;
	  obj->parent = current_obj;
	  obj->dynamic_state = next_state;
	  obj->state = HybridNode::IN_OPEN_SET;
	  obj->parent = current_obj;
	  node_map_.insert(make_pair(obj->grid_pos, obj));
	  open_set_.push(obj);
	}
      }
      
}

bool HybridAstar::isValid(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i grid_p = posToGrid(pos);
  for(int i=0; i<body_.size(); i++)
  {
    Eigen::Vector3i body_grid(body_[i](0)+grid_p(0), 
		     body_[i](1)+grid_p(1),body_[i](2)+grid_p(2));
    auto it = obstacle_map_.find(body_grid);
    if (!(it==obstacle_map_.end()))
    {
      return false;
    }
  }
  
  return true;
}


HybridNode* HybridAstar::findInCloseset(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i grid_p = posToGrid(pos);
  auto it = node_map_.find(grid_p);
  if(it == node_map_.end())
    return nullptr;
  return it->second->state == HybridNode::IN_CLOSE_SET ? it->second : nullptr;
}

HybridNode* HybridAstar::findInOpenset(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i grid_p = posToGrid(pos);
  auto it = node_map_.find(grid_p);
  if(it == node_map_.end())
    return nullptr;
  return it->second->state == HybridNode::IN_OPEN_SET ? it->second : nullptr;
}


vector< Eigen::Vector3d > HybridAstar::getTrajectory(double dt)
{
  
}

void HybridAstar::getNextState(Eigen::Matrix< double, int(6), int(1) >& state0, 
				Eigen::Matrix< double, int(6), int(1) >& state1, 
				Eigen::Vector3d u, double tau)
{
  for (int i = 0; i < 3; ++i)
    A_mat_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * u;
  integral.tail(3) = tau * u;

  state1 = A_mat_ * state0 + integral;
  
  Eigen::Vector3d stail = state1.tail(3);
  
}

vector< Eigen::Vector3d > HybridAstar::getTrajPoints()
{
  std::vector<Eigen::Vector3d> state_list;
  double dt = 0.04;

  HybridNode* node = path_node_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Eigen::Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->dynamic_state;

    for (double t = duration; t >= -1e-5; t -= dt)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  return state_list;
}


double HybridAstar::getHeuristic(const Eigen::VectorXd& stage0, const Eigen::VectorXd& stage1, double& optimal_time)
{
  //OPVP as heu.

  Eigen::Vector3d p0 = stage0.head(3);
  Eigen::Vector3d p1 = stage1.head(3);
  
  Eigen::Vector3d v0 = stage0.tail(3);
  Eigen::Vector3d v1 = stage1.tail(3);
  
  //std::cout<<"XXX1: "<<stage0<<std::endl;
  //std::cout<<"XXX2: "<<stage1<<std::endl;
  
  Eigen::Vector3d param1 = p1-p0;
  Eigen::Vector3d param2 = v0+v1;
  double param3 = v1.dot(v1) + v0.dot(v1) + v0.dot(v0);
  
  double coef1 = param1.dot(param1);
  double coef2 = (param2).dot(param1);
  double coef3 = param3;
  
  double c1 = -36.0 * coef1;
  double c2 = 24.0 * coef2;
  double c3 = -4.0 * coef3;
  double c4 = 0;
  double c5 = 1;
  
  //std::cout<<"C1: "<<c1<<", "<<c2<<", "<<c3 <<std::endl;
  
  Eigen::Matrix4d Mx;
  Mx << 0, -c3, -c2, -c1,
	1,   0,   0,   0,
	0,   1,   0,   0,
	0,   0,   1,   0;
  
  Eigen::EigenSolver<Eigen::MatrixXd> es(Mx);
  auto roots = es.eigenvalues();
  optimal_time = -1;
  double cost = 100000;
  
  double t_best = (p1 - p0).lpNorm<Eigen::Infinity>() / max_vel_;
  
  //std::cout<<"Best T:"<<t_best<<endl;
  for(int i=0; i<4; i++)
  {
    auto root = roots(i);
    if (std::fabs(root.imag()) > 0.00001 || root.real()<=0)
      continue;
    
    
    double T = root.real();
    
    //double J = T - ((c1)/(3*T*T*T)) - ((c2)/(2*T*T)) - ((c3)/(T));
    
    if (T < t_best)
      continue;
    
    double J = T + ((12*coef1)/(T*T*T)) + ((-12*coef2)/(T*T)) + ((4*coef3)/(T));
    
    if (J < cost)
    {
      cost = J;
      optimal_time = T;
    }
    
  }
  
  double T = t_best;
  double J = T + ((12*coef1)/(T*T*T)) + ((-12*coef2)/(T*T)) + ((4*coef3)/(T));
  
  if (J < cost)
  {
     cost = J;
     optimal_time = T;
  }
  
  if (cost <= 0)
  {
    LOG(ERROR)<<"CHECK Heu.";
  }
  
  
  return cost;
}

std::vector<Eigen::VectorXd> HybridAstar::getPath()
{
  vector<Eigen::VectorXd> path;
  for (int i = 0; i < path_node_.size(); ++i)
  {
    path.push_back(path_node_[i]->dynamic_state);
  }
  return path;
}


Eigen::Vector3i HybridAstar::posToGrid(const Eigen::Vector3d& n)
{
  return (n / resolution_).array().floor().cast<int>();
}
