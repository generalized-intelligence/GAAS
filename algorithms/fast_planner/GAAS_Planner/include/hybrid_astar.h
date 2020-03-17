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
#include "utils/scopetimer.h"

class HybridNode
{
public:
  Eigen::Vector3i grid_pos;
  Eigen::Matrix<double, 6, 1> dynamic_state;
  double G, H;
  Eigen::Vector3d input;
  double duration;
  HybridNode* parent;
  int state;
  
  enum
  {
    IN_OPEN_SET = 1,
    IN_CLOSE_SET = 2
  };
  
  HybridNode():G(0), H(0), duration(0), parent(nullptr), state(1)
  {
  }
};

class CompareHybrirdAstarNode
{
public:
  bool operator ()(const HybridNode* n1, const HybridNode* n2)
  {
    return (n1->G+n1->H) > (n2->G+n2->H);
  }
};

//Same as boost::hash_combine
//https://www.boost.org/doc/libs/1_33_1/doc/html/hash_combine.html
struct HybridCoordsHashFunc
{
  std::size_t operator()(const Eigen::Vector3i &v) const
  {
    std::size_t seed = 0;
    for (int i=1; i<3; i++)
      seed ^= std::hash<int>()(v(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);	//From hash_combine.
    
    return seed; 
  }
};

class HybridAstar
{
private:
  std::priority_queue<HybridNode*, std::vector<HybridNode*>, CompareHybrirdAstarNode> open_set_;
  std::unordered_map<Eigen::Vector3i, HybridNode*, HybridCoordsHashFunc> node_map_;
  std::vector<HybridNode*> close_set_;
  
  std::unordered_map<Eigen::Vector3i, Eigen::Vector3d , HybridCoordsHashFunc> obstacle_map_;
  
  //std::set<Eigen::Vector3d> obstacle_map_; //TODO:Use sdf_map
  std::vector<HybridNode* > path_node_;
  
  std::vector<Eigen::Vector3d> movement_list_;
  
  Eigen::Vector3d end_pt_, start_pt_, start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 1> end_state_;
  
  std::vector<Eigen::Vector3d> body_;
  
  double max_vel_;
  double max_acc_;
  double lambda_h_;
  double margin_;
  double tie_;
  double radius_;
  double resolution_;
  int margin_g_, radius_g_;  //How many grids.
  
  Eigen::Matrix<double, 6, 6> A_mat_;
  
  void getPathFromNode(HybridNode* end_node);
  
  double getHeuristic(const Eigen::VectorXd &stage0, const Eigen::VectorXd &stage1, double& optimal_time);
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);
  void extendRound( HybridNode* current_obj);
  
  Eigen::Vector3i posToGrid(const Eigen::Vector3d &n);
  
  
  HybridNode* findInCloseset(const Eigen::Vector3d &pos);
  HybridNode* findInOpenset(const Eigen::Vector3d &pos);
  
  
  bool isValid(const Eigen::Vector3d &pos);
  
  ScopeTimer *time1;
  ScopeTimer *time2;

  
public:
  HybridAstar();
  ~HybridAstar();
  void setParam();
  void reset();
  
  std::vector<Eigen::VectorXd> getPath();
  std::vector<Eigen::Vector3d> getTrajPoints();
  Eigen::MatrixXd getSampleMatrix(double &dt);
  
  
  int findPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
  
  void setObstacle(std::vector<Eigen::Vector3d> &obstacle_list);	//TODO:use SDF
  std::vector<Eigen::Vector3d> getTrajectory(double dt);
  
  
  typedef std::shared_ptr<HybridAstar> Ptr;
  
};

#endif // __HYBRID_ASTAR_H_
