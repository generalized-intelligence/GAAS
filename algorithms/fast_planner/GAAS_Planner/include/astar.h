#ifndef __ASTAR_H_
#define __ASTAR_H_

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>
#include <glog/logging.h>
#include <vector>
#include <queue>
#include <set>
#include "plan_enviroment/discrete_grid.h"
#include <memory>

class Node
{
public:
  Eigen::Vector3d position;
  double G, H;
  Node* parent;
  
  Node()
  {
    parent = nullptr;
  }
};

class CompareAstarNode
{
public:
  bool operator ()(Node* n1, Node* n2)
  {
    return n1->G+n1->H > n2->G+n2->H;
  }
};


class Astar
{
private:
  std::priority_queue<Node*, std::vector<Node*>, CompareAstarNode> open_set_;
  std::set<Node*> close_set_;
  
  int obstacle_map_; //TODO:Use sdf_map
  
  Eigen::Vector3d end_pt_, start_pt_;
  void extend_round(Node* current_obj);
  
public:
  Astar();
  ~Astar();
  void setParam();
  void reset();
  
  int find_path(Eigen::start_pt, Eigen::end_pt);
  void set_obstacle(DiscreteGrid &dg_map);
  
  typedef std::shared_ptr<Astar> Ptr;
  
};

#endif // __ASTAR_H_
