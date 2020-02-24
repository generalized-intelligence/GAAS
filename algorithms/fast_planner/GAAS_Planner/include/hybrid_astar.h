#ifndef __HYBRID_ASTAR_H_
#define __HYBRID_ASTAR_H_

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

class HybridNode
{
public:
  Eigen::Matrix<double, 6, 1> state;
  double G, F, H;
  Eigen::Vector3d input;
  double duration;
  HybridNode* parent;
  
  HybridNode():G(0), H(0), F(0), duration(0), parent(nullptr)
  {
  }
};

class CompareHybrirdAstarNode
{
public:
  bool operator ()(const Node* n1, const Node* n2)
  {
    return (n1->G+n1->H) > (n2->G+n2->H);
  }
};

class HybridAstar
{
private:
  std::priority_queue<HybridNode*, std::vector<HybridNode*>, CompareHybrirdAstarNode> open_set_;
  std::set<HybridNode*> close_set_;
  
  std::set<Eigen::Vector3d> obstacle_map_; //TODO:Use sdf_map
  std::vector<HybridNode* > path_node_;
  
  std::vector<Eigen::Vector3d> movement_list_;
  
  Eigen::Vector3d end_pt_, start_pt_, start_vel_, end_vel_, start_acc_;
  
  std::vector<Eigen::Vector3d> body_;
  
  double max_vel_;
  double max_acc_;
  double lambda_h_;
  double margin_;
  double tie_;
  double radius_;
  double resolution;
  
  voit getPathFromNode(HybridNode* end_node);
  
  double getHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);
  void extendRound(const HybridNode* current_obj);
  
public:
  HybridAstar();
  ~HybridAstar();
  void setParam(cv::FileStorage &config);
  void reset();
  
  int findPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
  
  void setObstacle(std::set<Eigen::Vector3d> &obstacle_map);	//TODO:use SDF
  std::vector<Eigen::Vector3d> getTrajectory(double dt);
  
  
  typedef std::shared_ptr<HybridAstar> Ptr;
  
};

#endif // __HYBRID_ASTAR_H_
