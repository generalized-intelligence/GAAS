#include "G2OTypes.h"
#include <memory>
#include <iostream>

using namespace std;

//struct State//the prev-state of drone.
//{
//    
//};
typedef VertexPR State;


class GlobalOptimizationGraph
{
public:
    GlobalOptimizationGraph(const std::string& config_file_path);
    void addBlockAHRS();
    void addBlockSLAM();
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();
    void doOptimization();


private:
    std::vector<State> historyStates;
    State currentState;
    G2OOptimizationGraph graph;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg *solver;

    int EdgeID = 0;
    int VertexID = 0;
};
GlobalOptimizationGraph::GlobalOptimizationGraph()
{

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    currentState.setEstimate(currentState);
    currentState.setId(0);//vertex 0 in optimization graph.
    optimizer.addVertex(&currentState);
}
GlobalOptimizationGraph::addBlockSLAM()
{
    pEdgeSlam = new EdgeAttitude();
    pEdgeSlam->setId(this->EdgeID);
    this->EdgeID++;
    pEdgeSlam->setMeasurement(...);
    pEdgeSlam->setInformation();
    optimizer.addEdge(pEdgeSlam);
}
GlobalOptimizationGraph::addBlockQRCode()
{
    pEdgeQRCode = 
}
GlobalOptimizationGraph::addBlockSceneRetriever()
{
    pBlockSceneRetriever = 
}
GlobalOptimizationGraph::addBlockFCAttitude()
{
    pEdgeAttitude = ...
}
GlobalOptimizationGraph::addBlockAHRS()
{
    pEdgeAHRS = ....
    if(check_avail())
    {
        ...
    }
    else
    {
        pEdgeAHRS->setLevel(1);
    }

}
GlobalOptimizationGraph::doOptimization()
{
    
    this->optimizer.initializeOptimization();
    this->optimizer.optimize(10);
    this->historyStates.push_back(currentState);
    this->historyStates.reserve();///...
}










