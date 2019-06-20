#ifndef COORDINATE_MANAGER_H
#define COORDINATE_MANAGER_H

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimizable_graph.h>
#include "G2OTypes.h"
#include "G2OTypes_EdgeSLAMPRV.h"
#include <opencv2/core/persistence.hpp>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "GPSExpand.h"
#include "CallbacksBufferBlock.h"
#include <cmath>
#include <deque>
#include <opencv2/opencv.hpp>
#include "GOG_Frame.h"

class CoordinateManager
{
public:
    CoordinateManager(vector<shared_ptr<Scene>>& p_scene_vec)
    {
    }
    CoordinateManager(const string& scene_file_list_path)
    {
        std::ifstream ifstr_json( scene_file_list_path);
        json json_obj;
        ifstr_json>>json_obj;
        auto scenes = json_obj["scenes"];
        json::iterator it; 
        for(it = scenes.begin();it!=scenes.end();++it)
        {
            auto scene_obj = it.value();
            std::string scene_file_path(scene_obj["scene_path"].get<string>());   
            shared_ptr<SceneRetriever> pSR( new SceneRetriever(voc_path,scene_file_path) );
            double lon,lat;
            lon = scene_obj["gps_longitude"].get<double>();
            lat = scene_obj["gps_latitude"].get<double>();
            shared_ptr<MultiSceneNode> pNode(new MultiSceneNode(pSR,lon,lat));
            this->insertSceneIntoKDTree(pNode);//gps indexed!
            if (scene_obj.find("sfm_model_path")!=scene_obj.end())
            {
                pNode->setSfMModelPath(scene_obj["sfm_model_path"].get<string>());
            }
        }
    }
    std::pair<Vector3d,Matrix3d> local_to_local_transform(int scene_index,Vector3d xyz,Matrix3d rotation);
};


#endif
