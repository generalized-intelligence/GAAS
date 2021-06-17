//
// Created by gishr on 2019/10/11.
//

#ifndef SCENE_RETRIEVING_WORLD_H
#define SCENE_RETRIEVING_WORLD_H

#include "building.h"
#include <geometry_msgs/PoseStamped.h>

#include <mutex>

using namespace std;

class World {

public:

    World(string config_path);

    // NOTE it is different from "Path Finding"
    vector<geometry_msgs::PoseStamped> FindWayPoints(geometry_msgs::PoseStamped& mavros_pose,
                                                     geometry_msgs::PoseStamped& target_pose);

    template<class T>
    inline bool isInBuilding(T& point, int& building_idx)
    {
        for(int i=0; i<mBuildings.size(); i++)
        {
            if(mBuildings[i]->isInBuilding(point))
            {
                building_idx = i;
                return true;
            }
        }
        return false;
    }

    inline Eigen::Vector3f Pose2Eigen(geometry_msgs::PoseStamped& pose)
    {
        Eigen::Vector3f eigen(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        return eigen;
    }

    inline geometry_msgs::PoseStamped Eigen2Pose(Eigen::Vector3f& position, string frame_id="map")
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = position[0];
        pose.pose.position.y = position[1];
        pose.pose.position.z = position[2];

        return pose;
    }


public:
    vector<shared_ptr<Building>> mBuildings;

private:

    int mBuildingNum = -1;

};


#endif //SCENE_RETRIEVING_MAP_H
