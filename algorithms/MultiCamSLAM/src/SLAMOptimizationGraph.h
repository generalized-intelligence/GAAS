#ifndef SLAM_OPTIMIZATION_GRAPH_H
#define SLAM_OPTIMIZATION_GRAPH_H
#include <glog/logging.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/slam/dataset.h> 
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/SmartFactorParams.h> //like chi2 outlier select.

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>


#include <opencv2/core/persistence.hpp>

using namespace gtsam;
using namespace std;


class SLAMOptimizationGraph
{
public:
    SLAMOptimizationGraph(FileStorage fSettings)
    {
        //...
    }

    void initCams()
    {
        noiseModel bind_model(0,0,0,0,0,0);
        //Symbol('c',0) stands for original pose.
        for(caminfo = CamInfoList[i],.....)
        {
            Pose3 CamPose = ....
            graph.addPrior<BetweenFactor>(Symbol('c',i),Symbol('c',0),CamPose,bind_model);
            
        }

    }
    void insertFrame()
    {
        noiseModel bind_model(0,0,0,0,0,0);
        //Symbol('c',0) stands for original pose.
        for(caminfo = CamInfoList[i],.....)
        {
            Pose3 CamPose = ....
            graph.addPrior<BetweenFactor>(Symbol('c',i+cam_num*frame_index),Symbol('c',0),CamPose,bind_model);
            
        }

    }
    void addPose();
    void addPoint3d();
    void addPoint2d_3d_projection();

    //
    void addCamObservation(int cam_index,vector<...> p2ds,vector<...> p3ds)
    {
        graph.emplace_shared(GeneralICPFactor<Pose3,Point3>(Symbol('c',cam_index + cam_num*frame_index),p3d,Noise ....));//bind with relative camera.
    }
private:
    NonlinearFactorGraph graph;
    ISAM2 *p_isam;
    FileStorage& fSettings;
};





#endif
