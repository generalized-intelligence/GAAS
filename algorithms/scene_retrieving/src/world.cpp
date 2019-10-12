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
    Eigen::Vector3f origin(5,0,0);
    Building test_building(origin, 0,
                           door, 0);
    Eigen::Vector3f origin2(5,15,0);
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

vector<geometry_msgs::PoseStamped> World::FindWayPoints(geometry_msgs::PoseStamped& mavros_pose, geometry_msgs::PoseStamped& target_pose)
{

    LOG(INFO)<<"FindWayPoints!"<<endl;
    LOG(INFO)<<"mavros_pose: "<<mavros_pose<<endl;
    LOG(INFO)<<"target pose: "<<target_pose<<endl;

    vector<geometry_msgs::PoseStamped> path;
    Eigen::Vector3f safe_point(0, 0, 3);
    geometry_msgs::PoseStamped safe_point_pose = Eigen2Pose(safe_point);

    // TODO: There is no obstacle avoidance at the moment, and I am using a very simple logic to control the drone
    // we can introduce obstacle avoidance to it.

    // case 1, drone not in any building, target in building
    // wps: safe_point -> building door -> building point
    int building_index = -1;
    if(!isInBuilding(mavros_pose, building_index))
    {
        if(isInBuilding(target_pose, building_index))
        {
            Eigen::Vector3f door_position = mBuildings[building_index].DoorGlobalPosition();
            geometry_msgs::PoseStamped door = Eigen2Pose(door_position);
            path.push_back(safe_point_pose);
            path.push_back(door);
            path.push_back(target_pose);
            LOG(INFO)<<"door: "<<door<<endl;
            LOG(INFO)<<"target: "<<target_pose<<endl;
            LOG(INFO)<<"Drone not in any building, target in building!"<<endl;
            return path;
        }
    }

    // case 2, drone in buiding, target not in building
    // wps: current building door -> safe point -> target pose
    int building_index_2 = -1;
    if(!isInBuilding(target_pose, building_index_2))
    {
        if(isInBuilding(mavros_pose, building_index_2))
        {
            Eigen::Vector3f door_position = mBuildings[building_index_2].DoorGlobalPosition();
            geometry_msgs::PoseStamped door = Eigen2Pose(door_position);
            path.push_back(door);
            path.push_back(safe_point_pose);
            path.push_back(target_pose);
            LOG(INFO)<<"door: "<<door<<endl;
            LOG(INFO)<<"target: "<<target_pose<<endl;
            LOG(INFO)<<"Drone in building, target not in building!"<<endl;
            return path;
        }
    }

    // case 3, drone in building, target in another building
    // wps: current building door -> safe_point -> target building door -> target building position
    int building_index_3 = -1;
    int building_index_4 = -1;
    if(isInBuilding(mavros_pose, building_index_3))
    {
        if(isInBuilding(target_pose, building_index_4))
        {
            // go out of current door
            Eigen::Vector3f current_door_position = mBuildings[building_index_3].DoorGlobalPosition();
            geometry_msgs::PoseStamped door = Eigen2Pose(current_door_position);
            path.push_back(door);

            // go to a safe point before going to the next door
            path.push_back(safe_point_pose);

            // go to the target building door
            Eigen::Vector3f target_door_position = mBuildings[building_index_4].DoorGlobalPosition();
            geometry_msgs::PoseStamped target_door = Eigen2Pose(target_door_position);
            path.push_back(target_door);

            // go to the target from the target building door
            path.push_back(target_pose);

            LOG(INFO)<<"door: "<<door<<endl;
            LOG(INFO)<<"target: "<<target_pose<<endl;
            LOG(INFO)<<"Drone in building, target in building!"<<endl;
            return path;
        }
    }

    // case 4, drone not in any building, target not in any building
    // wps : safe point -> target
    int building_index_5 = -1;
    if(!isInBuilding(mavros_pose, building_index))
    {
        if(!isInBuilding(target_pose, building_index))
        {
            path.push_back(safe_point_pose);
            path.push_back(target_pose);
            LOG(INFO)<<"Drone not in building, target not in building!"<<endl;
            return path;
        }
    }

    LOG(WARNING)<<"It should have been all possible cases, but I don't know why I can see this log!"<<endl;

}

