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
#include <opencv2/opencv.hpp>
#include <cmath>

class Node
{
public:
  Eigen::Vector3d position;
  double G, H;
  Node* parent;
  
  Node():G(0), H(0), parent(nullptr),position()
  {
  }
};

class CompareAstarNode
{
public:
  bool operator ()(const Node* n1, const Node* n2)
  {
    return (n1->G+n1->H) > (n2->G+n2->H);
  }
};


class Astar
{
private:
  std::priority_queue<Node*, std::vector<Node*>, CompareAstarNode> open_set_;
  std::set<Node*> close_set_;
  
  std::set<Eigen::Vector3d> obstacle_map_; //TODO:Use sdf_map
  std::vector<Node* > path_node_;
  
  std::vector<Eigen::Vector3d> movement_list_;
  
  Eigen::Vector3d end_pt_, start_pt_;
  
  std::vector<Eigen::Vector3d> body_;
  
  
  double radius_;	//Drone radius. m
  double margin_;	//Drone center to the obstacle. m
  double resolution_;	//1 grid scale. m  
  int margin_g_, radius_g_;  //How many grids.
  
  void extendRound(const Node* current_obj);
  bool isValid(const Eigen::Vector3d &current_obj);
  Node* findInCloseset(const Eigen::Vector3d &pos);
  Node* findInOpenset(const Eigen::Vector3d &pos);
  
  double getMoveCost(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);
  
  double getManhattan(const Eigen::Vector3d n1, const Eigen::Vector3d n2);
  void getPathFromNode(Node* end_node);
  
  void gen6ConnectionMovement();
  void gen27ConnectionMovement();
  
  
public:
  Astar();
  ~Astar();
  void setParam(cv::FileStorage &astar_config);
  void reset();
  
  int findPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
  void setObstacle(std::set<Eigen::Vector3d> &obstacle_map);
  std::vector<Eigen::Vector3d> getPath();
  
  typedef std::shared_ptr<Astar> Ptr;
  
};

#endif // __ASTAR_H_
