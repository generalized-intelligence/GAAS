//
// Created by gishr on 2019/10/11.
//

#include "world.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

World::World()
{
    //for debugging
    Eigen::Vector3f door(0,3,0);
    Eigen::Vector3f origin(0,0,0);
    Building test_building(origin, 0,
                           door, 0);
    Eigen::Vector3f origin2(0,15,0);
    Building test_building2(origin2, test_building.PointsNum(),
                            door, test_building.DoorPtsNum());


    mBuildings.emplace_back(test_building);
    mBuildings.emplace_back(test_building2);

    //coordinate used by this world
    /*
     *                 +x
     *                 ^
     *                 |
     *                 |
     *                 |
     *  +y <-----------|0
     */
}

vector<geometry_msgs::PoseStamped> World::FindPath(geometry_msgs::PoseStamped& mavros_pose, geometry_msgs::PoseStamped& target_pose)
{
    vector<geometry_msgs::PoseStamped> path;

    // case 1, drone not in any building, target in building
    int building_index = -1;
    if(!isInBuilding(mavros_pose, building_index))
    {
        if(isInBuilding(target_pose, building_index))
        {
            Eigen::Vector3f door_position = mBuildings[building_index].DoorGlobalPosition();
            geometry_msgs::PoseStamped door = Eigen2Pose(door_position);
            path.push_back(door);
            path.push_back(target_pose);
            return path;
        }
    }

    // case 2, drone in buiding, target not in building
    int building_index_2 = -1;
    if(!isInBuilding(mavros_pose, building_index_2))
    {
        if(isInBuilding(target_pose, building_index_2))
        {
            Eigen::Vector3f door_position = mBuildings[building_index_2].DoorGlobalPosition();
            geometry_msgs::PoseStamped door = Eigen2Pose(door_position);
            path.push_back(door);
            path.push_back(target_pose);
            return path;
        }
    }

}

