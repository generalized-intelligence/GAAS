#include "astar.h"
#include <stdio.h>
Astar::Astar()
{
  
}

Astar::~Astar()
{

}

void Astar::setParam(cv::FileStorage& astar_config)
{
  //TODO:Use config file.
  resolution_ = 0.1;
  margin_ = 0.6; //use_resolution
  radius_ = 0.4;
  
  margin_g_ = (int)(margin_ / resolution_) + 1;
  radius_g_ = (int)(radius_/ resolution_) + 1;
  
  bool use_6_connection = true;
  
  if (use_6_connection)
    gen6ConnectionMovement();
  
  body_ = Eigen::MatrixXd(3, margin_g_*margin_g_);
  for(int i=-margin_g_; i<=margin_g_; i+=1)
    for(int j=-margin_g_; j<=margin_g_; j+=1)
      for(int k=-margin_g_; k<=margin_g_; k+=1)
      {
	body_.push_back(Eigen::Vector3d(i,j,k));
      }
}

void Astar::reset()
{
    std::priority_queue<Node*, std::vector<Node*>, CompareAstarNode> empty;
    open_set_.swap(empty);
  
    close_set_.clear();
    obstacle_map_.clear();
    path_node_.clear();
    movement_list_.clear();
    end_pt_ = nullptr;
    start_pt_ = nullptr;
    body_.clear();
}

void Astar::setObstacle(const set< Eigen::Vector3d >& obstacle_map)
{
    obstacle_map_ = obstacle_map;
}

int Astar::findPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt)
{
    start_pt_ = start_pt;
    end_pt_ = end_pt;
    CHECK(is_valid(start_pt)) << "The Start point is in obstacle, Check enviroment please.";
    CHECK(is_valid(end_pt)) << "The End point is in obstacle, Check order please.";
    
    Node* current_node = new Node();
    current_node->position = start_pt;
    current_node->H = getManhattan(current_node->position, end_pt);
    open_set_.push(current_node);
    
    while(!open_set_.empty())
    {
      current_node = open_set_.top();
      if ((end_pt-current_node->position).norm() <= 1)
      {
	getPathFromNode();
	return 1;
      }
      open_set_.pop();
      close_set_.insert(current_node);
      extendRound(current_node);
    }
}

double Astar::getManhattan(const Eigen::Vector3d n1, const Eigen::Vector3d n2)
{
  double lambda = 10;
  return lambda*(n1-n2).norm()*(n1-n2).norm();
}


void Astar::extendRound(const Node* current_obj)
{
  for (int i=0; i<movement_list_.size();i++)
  {
    Eigen::Vector3d next_pos(current_obj->position(0)+movement_list_[i](0), 
				current_obj->position(1)+movement_list_[i](1), 
				current_obj->position(2)+movement_list_[i](2));
    if (!isValid(next_pos))
      continue;
    
    if (!(findInCloseset(next_pos) == nullptr))
	continue;
    Node * obj = findInOpenset(next_pos);
    if (!obj==nullptr)
    {
      int new_g = current_obj->G+ getMoveCost(current_obj->position, next_pos);
      if (obj->G > new_g)
      {
	obj->G = new_g;
	obj->parent = current_obj;
      }
    }
    else
    {
      Node *new_obj = new Node();
      new_obj->position = next_pos;
      new_obj->G = current_obj->G + getMoveCost(current_obj->position, next_pos);
      new_obj->H = getManhattan(next_pos, end_pt_);
      new_obj->parent = current_obj;
      open_set_.push(new_obj);
    }
  }
}

bool Astar::isValid(const Eigen::Vector3d &current_obj)
{
  std::set<Eigen::Vector3d> drone_set;
  drone_set.clear();
  std::set<Eigen::Vector3d> temp_set;
  temp_set.clear();
  for(int i=0; i<body_.size(); i++)
  {
    drone_set.insert(body_[i](0)+current_obj(0), 
		     body_[i](1)+current_obj(1),body_[i](2)+current_obj(2));
  }
  std::set_intersection(drone_set.begin(),drone_set.end(),
			  obstacle_map_.begin(),obstacle_map_.end(),
			  std::inserter(temp_set, temp_set.begin()));
  if(!temp_set.empty())
    return false;
  return true;
}

Node* Astar::findInCloseset(const Eigen::Vector3d& pos)
{
  for(std::set<Node*>::iterator it=close_set_.begin();it!=close_set_.end();it++)
  {
    if (it->position == pos)
      return it;
  }
  return nullptr;
}

Node* Astar::findInOpenset(const Eigen::Vector3d& pos)
{
  std::priority_queue<Node*, std::vector<Node*>, CompareAstarNode> temp_set = open_set_;
  Node* obj;
  while(!temp_set.empty())
  {
    obj = temp_set.top();
    if (obj->position == pos)
      return obj;
    temp_set.pop();
  }
  return nullptr;
}

double Astar::getMoveCost(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  if ((pos1-pos2).norm() == 1)
  {
    if (pos1(2) == pos2(2))
      return 1;
    else
      return 10;
  }
  else
  {
    DLOG(INFO) << "[Astar] - move_cost - norm is: "<<(pos1-pos2).norm();
    return (pos1-pos2).norm();
  }
}



void Astar::getPathFromNode(Node* end_node)
{
  Node* current_obj = end_node;
  path_node_.push_back(end_node);
  while (current_obj->parent != NULL)
  {
    current_obj = current_obj->parent;
    path_node_.push_back(current_obj);
  }
 std::reverse(path_node_.begin(), path_node_.end());
}

void Astar::gen6ConnectionMovement()
{
  movement_list_.push_back(new Eigen::Vector3d(-1,0,0));
  movement_list_.push_back(new Eigen::Vector3d(1,0,0));
  movement_list_.push_back(new Eigen::Vector3d(0,-1,0));
  movement_list_.push_back(new Eigen::Vector3d(0,1,0));
  movement_list_.push_back(new Eigen::Vector3d(0,0,-1));
  movement_list_.push_back(new Eigen::Vector3d(0,0,1));
}

void Astar::gen27ConnectionMovement()
{
  for (int i=-1; i<=1; i+=1)
    for (int j=-1; j<=1; j+=1)
      for (int k=-1; k<=1; k+=1)
      {
	if (i==0 && j==0 && k==0)
	  continue;
	movement_list_.push_back(new Eigen::Vector3d(i,j,k));
      }
}

std::vector< Eigen::Vector3d > Astar::getPath()
{
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < path_node_.size(); ++i)
  {
    path.push_back(path_node_[i]->position);
  }
  return path;
}




