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
#include <map>

class Node
{
public:
  
  Eigen::Vector3i grid_pos;
  Eigen::Vector3d position;
  double G, H;
  Node* parent;
  int state;
  
  enum
  {
    IN_OPEN_SET = 1,
    IN_CLOSE_SET = 2
  };
  
  Node():G(0), H(0), parent(nullptr),position(),grid_pos(),state(1)
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


//Same as boost::hash_combine
//https://www.boost.org/doc/libs/1_33_1/doc/html/hash_combine.html
struct CoordsHashFunc
{
  std::size_t operator()(const Eigen::Vector3i &v) const
  {
    std::size_t seed = 0;
    for (int i=1; i<3; i++)
      seed ^= std::hash<int>()(v(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);	//From hash_combine.
    
    return seed; 
  }
};


class Astar
{
private:
  std::priority_queue<Node*, std::vector<Node*>, CompareAstarNode> open_set_;
  std::set<Node*> close_set_;
  std::unordered_map<Eigen::Vector3i, Node* , CoordsHashFunc> node_map_;
  
  std::set<Eigen::Vector3d> obstacle_map_; //TODO:Use sdf_map
  std::vector<Node* > path_node_;
  
  std::vector<Eigen::Vector3d> movement_list_;
  
  Eigen::Vector3d end_pt_, start_pt_;
  
  std::vector<Eigen::Vector3d> body_;
  
  
  double radius_;	//Drone radius. m
  double margin_;	//Drone center to the obstacle. m
  double resolution_;	//1 grid scale. m  
  int margin_g_, radius_g_;  //How many grids.
  double tie_;	//H = tie_ * heu.
  
  void extendRound(const Node* current_obj);
  bool isValid(const Eigen::Vector3d &pos);
  Node* findInCloseset(const Eigen::Vector3d &pos);
  Node* findInOpenset(const Eigen::Vector3d &pos);
  
  double getMoveCost(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);
  
  double getManhattan(const Eigen::Vector3d n1, const Eigen::Vector3d n2);
  double getDiag(const Eigen::Vector3d n1, const Eigen::Vector3d n2);
  void getPathFromNode(Node* end_node);
  
  void gen6ConnectionMovement();
  void gen27ConnectionMovement();
  
  Eigen::Vector3i posToGrid(const Eigen::Vector3d &n);
  
  
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
