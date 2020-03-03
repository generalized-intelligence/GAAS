#include "hybrid_astar.h"


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
  close_set_.clear();
  obstacle_map_.clear();
  path_node_.clear();
  body_.clear();
  movement_list_.clear();
  end_pt_ = nullptr;
  start_pt_ = nullptr;
}

void HybridAstar::setParam(cv::FileStorage& config)
{
  //TODO:use cv config.
  resolution_ = 0.1;
  margin_ = 0.6; //use_resolution
  radius_ = 0.4;
  tie_ = 1.001;
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

voit HybridAstar::getPathFromNode(HybridNode* end_node)
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

void HybridAstar::setObstacle(set< Eigen::Vector3d >& obstacle_map)
{
  obstacle_map_ = obstacle_map;
}

int HybridAstar::findPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel)
{
  start_pt_ = start_pt;
  start_vel_ = start_vel;
  start_acc_ = start_acc;
  end_pt_ = end_pt;
  end_vel_ = end_vel;
  
  end_state_.head(3) = end_pt_;
  end_state_.tail(3) = end_vel_;
  
  HybridNode* current_node = new HybridNode();
  current_node->parent = nullptr;
  current_node->dynamic_state.head(3) = start_pt;
  current_node->dynamic_state.tail(3) = start_vel;
  current_node->grid_pos = posToGrid(start_pt);
  
  double time_to_goal;
  current_node->H = tie_* getHeuristic(current_node->dynamic_state, end_state_, time_to_goal);
  current_node->state = HybridNode::IN_OPEN_SET;
  
  open_set_.push(current_node);
  
  node_map_.insert(current_node->grid_pos, current_node);
  
  while(!open_set_.empty())
  {
    current_node = open_set_.top();
    Eigen::Vector3d current_pos = current_node->dynamic_state.head(3);
    if((end_pt_-current_pos).norm() <= resolution_)
    {
      getPathFromNode(current_node);
      return 1;
    }
    
    open_set_.pop();
    current_node->state = HybridNode::IN_CLOSE_SET;
    close_set_.insert(current_node);
    extendRound(current_node);
  }
}

void HybridAstar::extendRound(const HybridNode* current_obj)
{
  double T = 0.8;
  Eigen::Matrix<double, 6, 1> current_state = current_obj->dynamic_state;
  Eigen::Matrix<double, 6, 1> next_state;
  for (double ax = -max_acc_; ax <= max_acc_; ax+=0.5*max_acc_)
    for (double ay = -max_acc_; ay <= max_acc_; ay+=0.5*max_acc_)
      for (double az = -max_acc_; az <= max_acc_; az+=0.5*max_acc_)
      {
	Eigen::Vector3d input_a(ax, ay, 0.5*az);
	stateTransit(current_state, next_state, input_a, T);
	
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
	    is_collision = false;
	    break;
	  }
	}
	
	if(is_collision==true)
	  continue;
	
	double new_g=0, new_h=0, time_to_end;
	new_g = (input_a.squaredNorm()+1)*T + current_obj->G;
	new_h = tie_*getHeuristic(next_state, end_state_, time_to_end);
	
	
	HybridNode* obj = findInOpenset(next_pos);
	if (!obj == nullptr)
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
	  open_set_.push(obj);
	}
      }
      
}

bool HybridAstar::isValid(const Eigen::Vector3d& pos)
{

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

void HybridAstar::stateTransit(Eigen::Matrix< double, int(6), int(1) >& state0, 
				Eigen::Matrix< double, int(6), int(1) >& state1, 
				Eigen::Vector3d u, double tau)
{
  for (int i = 0; i < 3; ++i)
    A_mat_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * u;
  integral.tail(3) = tau * u;

  state1 = A_mat_ * state0 + integral;
}

double HybridAstar::getHeuristic(const Eigen::VectorXd stage0, const Eigen::VectorXd stage1, 
				  double& optimal_time)
{
  //OPVP as heu.

  Eigen::Vector3d p0 = stage0.head(3);
  Eigen::Vector3d p1 = stage1.head(3);
  
  Eigen::Vector3d v0 = stage0.tail(3);
  Eigen::Vector3d v1 = stage1.tail(3);
  
  Eigen::Vector3d param1 = p1-p0;
  Eigen::Vector3d param2 = v0+v1;
  Eigen::Vector3d param3 = v1.dot(v1) + v0.dot(v1) + v0.dot(v0);
  
  double coef1 = param1.dot(param1);
  double coef2 = (param2).dot(param1);
  double coef3 = param3;
  
  double c1 = -36.0 * coef1;
  double c2 = 24.0 * coef2;
  double c3 = -4.0 * coef3;
  double c4 = 0;
  double c5 = 1;
  
  Eigen::Matrix4d Mx;
  Mx << 0, -c3, -c2, -c1,
	1,   0,   0,   0,
	0,   1,   0,   0,
	0,   0,   1,   0;
  
  Eigen::EigenSolver<Eigen::MatrixXd> es(Mx);
  auto roots = es.eigenvalues();
  optimal_time = -1;
  double cost = 100000;
  
  double t_best = (p1 - p0).lpNorm<Infinity>() / max_vel_;
  
  
  for(int i=0; i<4; i++)
  {
    auto root = roots(i);
    if (std::fabs(root.imag()) > 0.00001 || root.real()<=0)
      continue;
    
    
    double T = root.real();
    
    if (T < t_best)
      continue;
    
    double J = T + ((12*coef1)/(T*T*T)) + ((-12*coef2)/T*T) + ((4*coef3)/(T));
    
    if (J < cost)
    {
      cost = J;
      optimal_time = T;
    }
  }
  
  return cost;
}

Eigen::Vector3i HybridAstar::posToGrid(const Eigen::Vector3d& n)
{
  return (n / resolution_).array().floor().cast<int>();
}
