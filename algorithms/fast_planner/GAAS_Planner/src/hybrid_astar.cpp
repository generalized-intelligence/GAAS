#include "hybrid_astar.h"

HybridAstar::HybridAstar()
{

}

HybridAstar::~HybridAstar()
{

}

void HybridAstar::reset()
{

}

void HybridAstar::setParam(cv::FileStorage& config)
{

}

voit HybridAstar::getPathFromNode(HybridNode* end_node)
{

}
void HybridAstar::setObstacle(set< Eigen::Vector3d >& obstacle_map)
{

}


int HybridAstar::findPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel)
{

}

void HybridAstar::extendRound(const HybridNode* current_obj)
{

}

vector< Eigen::Vector3d > HybridAstar::getTrajectory(double dt)
{

}

void HybridAstar::stateTransit(Eigen::Matrix< double, int(6), int(1) >& state0, Eigen::Matrix< double, int(6), int(1) >& state1, Eigen::Vector3d um, double tau)
{

}

double HybridAstar::getHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{

}
