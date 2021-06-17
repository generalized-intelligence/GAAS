//
// Created by gishr on 2019/10/10.
//

#include "building.h"

Building::Building(float height)
{

    mBuildingSub = mNH.subscribe("/gi/flag_pub/pose", 100, &Building::BuildingPointCallback, this);

    mCornerMutex = make_shared<std::mutex>();

    LOG(INFO)<<"Building initializing!"<<endl;

    std::thread t(&Building::SetCornerPointsAndGate, this);
    t.detach();

    while(mCornerAndGate.size() <= 5 && !mFinished)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    // TODO, assuming the building is a regular rectangle
    mWidth = mCorner_2.pose.position.x - mCorner_1.pose.position.z;
    mLength = mCorner_1.pose.position.y - mCorner_0.pose.position.y;
    mHeight = height;
}


void Building::BuildingPointCallback(const geometry_msgs::PoseStamped& pose)
{
    mCornerMutex->lock();
    if(mCornerAndGate.size() <= 5)
    {
        mCornerAndGate.push_back(pose);
        LOG(WARNING)<<"Received Position: "<<pose.pose.position.x<<", "<<pose.pose.position.y<<", "<<pose.pose.position.z<<endl;
    }
    mCornerMutex->unlock();
}



