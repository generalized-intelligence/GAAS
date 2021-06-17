//
// Created by gishr on 2019/10/11.
//

#include "world.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

World::World(string config_path)
{

    cv::FileStorage fsSettings(config_path, cv::FileStorage::READ);
    mBuildingNum = fsSettings["Building_Number"];
    LOG(INFO)<<"mBuildingNum: "<<mBuildingNum<<endl;

    assert(int(mBuildingNum) > 0);

    shared_ptr<Building> test_building = nullptr;
    mBuildings.resize(int(mBuildingNum));
    for(int i=0; i<int(mBuildingNum); i++)
    {
        test_building = make_shared<Building>();
        mBuildings[i] = test_building;
    }


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
    vector<geometry_msgs::PoseStamped> path;
    Eigen::Vector3f safe_point(0, 0, 3);
    geometry_msgs::PoseStamped safe_point_pose = Eigen2Pose(safe_point);

    // TODO: There is no obstacle avoidance at the moment, and I am using a very simple logic to control the drone
    // the output could be used as the input of path planning start and end
    // we can introduce obstacle avoidance to it.

    // case 1, drone not in any building, target in building
    // wps: safe_point -> building door -> building point
    int building_index = -1;
    if(!isInBuilding(mavros_pose, building_index))
    {
        if(isInBuilding(target_pose, building_index))
        {
            geometry_msgs::PoseStamped door = mBuildings[building_index]->DoorGlobalPosition();
            path.push_back(safe_point_pose);
            path.push_back(door);
            path.push_back(target_pose);
            LOG(INFO)<<"door: "<<door<<endl;
            LOG(INFO)<<"target: "<<target_pose<<endl;
            LOG(WARNING)<<"Drone not in any building, target in building!"<<endl;
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
            geometry_msgs::PoseStamped door = mBuildings[building_index_2]->DoorGlobalPosition();
            path.push_back(door);
            path.push_back(safe_point_pose);
            path.push_back(target_pose);
            LOG(INFO)<<"door: "<<door<<endl;
            LOG(INFO)<<"target: "<<target_pose<<endl;
            LOG(WARNING)<<"Drone in building, target not in building!"<<endl;
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
            geometry_msgs::PoseStamped door = mBuildings[building_index_3]->DoorGlobalPosition();;
            path.push_back(door);

            // go to a safe point before going to the next door
            path.push_back(safe_point_pose);

            // go to the target building door
            geometry_msgs::PoseStamped target_door = mBuildings[building_index_4]->DoorGlobalPosition();;
            path.push_back(target_door);

            // go to the target from the target building door
            path.push_back(target_pose);

            LOG(INFO)<<"door: "<<door<<endl;
            LOG(INFO)<<"target: "<<target_pose<<endl;
            LOG(WARNING)<<"Drone in building, target in building!"<<endl;
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
            LOG(WARNING)<<"Drone not in building, target not in building!"<<endl;
            return path;
        }
    }

    LOG(WARNING)<<"Does not fit in any case, neglect!"<<endl;
}

