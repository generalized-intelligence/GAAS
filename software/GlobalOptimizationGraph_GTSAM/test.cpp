/**
 * @file Pose2SLAMExample.cpp
 * @brief 2D SLAM example
 * @date Nov 7, 2016
 * @author Jing Dong
 */

/**
 * A simple 2D pose-graph SLAM
 * The robot moves from x1 to x5, with odometry information between each pair. 
 * the robot moves 5 each step, and makes 90 deg right turns at x3 - x5
 * At x5, there is a *loop closure* between x2 is avaible
 * The graph strcuture is shown:
 * 
 *  p-x1 - x2 - x3
 *         |    |
 *         x5 - x4 
 */


// In planar cases we use Pose2 variables (x, y, theta) to represent the robot poses in SE(2)
#include <gtsam/geometry/Pose2.h>

// class for factor graph, a container of various factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// class for graph nodes values, a container of various geometric types
// here Values is used as a container of SE(2)
#include <gtsam/nonlinear/Values.h>

// symbol class is used to index varible in values
// e.g. pose varibles are generally indexed as 'x' + number, and landmarks as 'l' + numbers 
#include <gtsam/inference/Symbol.h>

// Factors used in this examples
// PriorFactor gives the prior distribution over a varible
// BetweenFactor gives odometry constraints
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// optimizer class, here we use Gauss-Newton
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the 
// (appoximated / linearized) marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>


using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // Create a factor graph container
  NonlinearFactorGraph graph;

  // Add a prior on the first pose, setting it to the origin
  // The prior is needed to fix/align the whole trajectory at world frame
  // A prior factor consists of a mean value and a noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));
  graph.add(PriorFactor<Pose2>(Symbol('x', 1), Pose2(0, 0, 0), priorModel));

  // odometry measurement noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  // robot makes 90 deg right turns at x3 - x5
  graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, -M_PI_2), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 3), Symbol('x', 4), Pose2(0, -5, -M_PI_2), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 4), Symbol('x', 5), Pose2(-5, 0, -M_PI_2), odomModel));

  // loop closure measurement noise model
  noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // Add the loop closure constraint
  graph.add(BetweenFactor<Pose2>(Symbol('x', 5), Symbol('x', 2), Pose2(0, 5, -M_PI_2), loopModel));
  
  // print factor graph
  graph.print("\nFactor Graph:\n"); 


  // initial varible values for the optimization
  // add random noise from ground truth values
  Values initials;
  initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
  initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
  initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -M_PI_2 - 0.2));
  initials.insert(Symbol('x', 4), Pose2(10.2, -5.0, -M_PI + 0.1));
  initials.insert(Symbol('x', 5), Pose2(5.1, -5.1, M_PI_2 - 0.1));
  
  // print initial values
  initials.print("\nInitial Values:\n"); 


  // Use Gauss-Newton method optimizes the initial values
  GaussNewtonParams parameters;
  
  // print per iteration
  parameters.setVerbosity("ERROR");
  
  // optimize!
  GaussNewtonOptimizer optimizer(graph, initials, parameters);
  Values results = optimizer.optimize();
  
  // print final values
  results.print("Final Result:\n");


  // Calculate marginal covariances for all poses
  Marginals marginals(graph, results);
  
  // print marginal covariances
  cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;
  cout << "x4 covariance:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
  cout << "x5 covariance:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;

  return 0;
}
