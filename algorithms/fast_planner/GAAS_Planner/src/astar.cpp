#include "astar.h"
#include <stdio.h>
Astar::Astar()
{
  
}

Astar::~Astar()
{

}

void Astar::setParam()
{
  std::cout<<"Set param Astar."<<std::endl;
  //TODO:Use config file.
  resolution_ = 0.2;
  margin_ = 0.6; //use_resolution
  radius_ = 0.4;
  tie_ = 1.001;
  
  margin_g_ = std::ceil(margin_ / resolution_);
  radius_g_ = std::ceil(radius_/ resolution_);
  
  
  
  bool use_6_connection = false;
  
  if (use_6_connection)
    gen6ConnectionMovement();
  else
    gen27ConnectionMovement();
  std::cout<<"margin_g_: "<< margin_g_<<std::endl;
  
  for(int i=-margin_g_; i<=margin_g_; i+=1)
    for(int j=-margin_g_; j<=margin_g_; j+=1)
      for(int k=-margin_g_; k<=margin_g_; k+=1)
      {
	body_.push_back(Eigen::Vector3d(i,j,k));
      }
   std::cout<<"Finish Set param Astar."<<std::endl;   
}

void Astar::reset()
{
    std::cout<<"Reset Astar."<<std::endl;
    std::priority_queue<Node*, std::vector<Node*>, CompareAstarNode> empty;
    open_set_.swap(empty);
  
    close_set_.clear();
    obstacle_map_.clear();
    path_node_.clear();
    movement_list_.clear();
    body_.clear();
    node_map_.clear();
    std::cout<<"Finish Reset Astar."<<std::endl;
}

void Astar::setObstacle(vector< Eigen::Vector3d >& obstacle_list)
{
  for(int i=0;i<obstacle_list.size();i++)
  {
    obstacle_map_.insert(make_pair(posToGrid(obstacle_list[i]), obstacle_list[i]));
  }
}


int Astar::findPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt)
{
    start_pt_ = start_pt;
    end_pt_ = end_pt;
    //CHECK(is_valid(start_pt)) << "The Start point is in obstacle, Check enviroment please.";
    //CHECK(is_valid(end_pt)) << "The End point is in obstacle, Check order please.";
    
    Node* current_node = new Node();
    current_node->position = start_pt;
    current_node->H = tie_* getDiag(current_node->position, end_pt);
    current_node->grid_pos = posToGrid(current_node->position);
    current_node->state = Node::IN_OPEN_SET;
    open_set_.push(current_node);
    node_map_.insert(std::make_pair(current_node->grid_pos, current_node));
    
    while(!open_set_.empty())
    {
      current_node = open_set_.top();
      if ((end_pt-current_node->position).norm() <= resolution_)
      {
	getPathFromNode(current_node);
	return 1;
      }
      open_set_.pop();
      current_node->state = Node::IN_CLOSE_SET;
      //close_set_.insert(current_node);
      extendRound(current_node);
    }
    
    return 0;
}

void Astar::extendRound(Node* current_obj)
{
  for (int i=0; i<movement_list_.size();i++)
  {
    Eigen::Vector3d next_pos(current_obj->position(0)+movement_list_[i](0), 
				current_obj->position(1)+movement_list_[i](1), 
				current_obj->position(2)+movement_list_[i](2));
    if (!(findInCloseset(next_pos) == nullptr))
	continue;
    
    if (!isValid(next_pos))
      continue;
    
    Node * obj = findInOpenset(next_pos);
    if (!(obj==nullptr))
    {
      int new_g = current_obj->G+ getMoveCost(current_obj->position, next_pos);
      if (obj->G > new_g)
      {
	obj->position = next_pos;
	obj->G = new_g;
	obj->parent = current_obj;
      }
    }
    else
    {
      Node *new_obj = new Node();
      new_obj->position = next_pos;
      new_obj->grid_pos = posToGrid(next_pos);
      new_obj->G = current_obj->G + getMoveCost(current_obj->position, next_pos);
      new_obj->H = tie_ * getDiag(next_pos, end_pt_);
      new_obj->parent = current_obj;
      new_obj->state = Node::IN_OPEN_SET;
      open_set_.push(new_obj);
      node_map_.insert(std::make_pair(new_obj->grid_pos, new_obj));
    }
  }
}

Eigen::Vector3i Astar::posToGrid(const Eigen::Vector3d& n)
{
  return (n / resolution_).array().floor().cast<int>();
}


double Astar::getManhattan(const Eigen::Vector3d n1, const Eigen::Vector3d n2)
{
  //double lambda = 10;
  double dx = fabs(n1(0) - n2(0));
  double dy = fabs(n1(1) - n2(1));
  double dz = fabs(n1(2) - n2(2));
  return dx + dy + dz;
}

double Astar::getDiag(const Eigen::Vector3d n1, const Eigen::Vector3d n2)
{
  double dx = fabs(n1(0) - n2(0));
  double dy = fabs(n1(1) - n2(1));
  double dz = fabs(n1(2) - n2(2));

  double heu;
  double dt = min(min(dx, dy), dz);
  
  if (dx == dt)
  {
    heu = sqrt(3.0)*dt + sqrt(2.0)*min(dy-dt, dz-dt) + abs(dy-dz);
  }
  if (dy == dt)
  {
    heu = sqrt(3.0)*dt + sqrt(2.0)*min(dx-dt, dz-dt) + abs(dx-dz);
  }
  if (dz == dt)
  {
    heu = sqrt(3.0)*dt + sqrt(2.0)*min(dx-dt, dy-dt) + abs(dx-dy);
  }
  return heu;
}


bool Astar::isValid(const Eigen::Vector3d &pos)
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

Node* Astar::findInCloseset(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i grid_p = posToGrid(pos);
  auto it = node_map_.find(grid_p);
  if(it == node_map_.end())
    return nullptr;
  return it->second->state == Node::IN_CLOSE_SET ? it->second : nullptr;
}

Node* Astar::findInOpenset(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i grid_p = posToGrid(pos);
  auto it = node_map_.find(grid_p);
  if(it == node_map_.end())
    return nullptr;
  return it->second->state == Node::IN_OPEN_SET ? it->second : nullptr;
}

double Astar::getMoveCost(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  Eigen::Vector3i grid_p1 = posToGrid(pos1);
  Eigen::Vector3i grid_p2 = posToGrid(pos2);
  
  double dist = (pos1-pos2).norm();
  
  if (dist == 0)
    DLOG(INFO) << "[Astar] - move_cost - norm is: "<<dist;
  
  return grid_p1(2) == grid_p2(2) ? dist : 10 * dist;
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
  movement_list_.push_back(Eigen::Vector3d(-resolution_,0,0));
  movement_list_.push_back(Eigen::Vector3d(resolution_,0,0));
  movement_list_.push_back(Eigen::Vector3d(0,-resolution_,0));
  movement_list_.push_back(Eigen::Vector3d(0,resolution_,0));
  movement_list_.push_back(Eigen::Vector3d(0,0,-resolution_));
  movement_list_.push_back(Eigen::Vector3d(0,0,resolution_));
}

void Astar::gen27ConnectionMovement()
{
  std::cout<<"Start gen movement."<<std::endl;
  for (double i=-resolution_; i<=resolution_; i+=resolution_)
    for (double j=-resolution_; j<=resolution_; j+=resolution_)
      for (double k=-resolution_; k<=resolution_; k+=resolution_)
      {
	
	if (i==0 && j==0 && k==0)
	  continue;
	movement_list_.push_back(Eigen::Vector3d(i,j,k));
      }
  std::cout<<"Finish gen movement."<<std::endl;
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




