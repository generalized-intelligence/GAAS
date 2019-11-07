#ifndef OPTIMIZER_UTILS_H
#define OPTIMIZER_UTILS_H
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/config.h> // for GTSAM_USE_TBB
using namespace gtsam;
//using namespace gtsam::symbol_shorthand;



double chi2_red(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& config,int& dof_out) {
  // Compute degrees of freedom (observations - variables)
  // In ocaml, +1 was added to the observations to account for the prior, but
  // the factor graph already includes a factor for the prior/equality constraint.
  //  double dof = graph.size() - config.size();
  int graph_dim = 0;
  for(const boost::shared_ptr<gtsam::NonlinearFactor>& nlf: graph) {
    graph_dim += (int)nlf->dim();
  }
  //double dof = double(graph_dim) - double(config.dim()); // kaess: changed to dim
  //dof_out = (int)dof;
  double dof = double(config.dim()); //here in isam2,we use estimation dof instead of graph.dof.
  dof_out = config.dim();
  return 2. * graph.error(config) / dof; // kaess: added factor 2, graph.error returns half of actual error
}

#endif
