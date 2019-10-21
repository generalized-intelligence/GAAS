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

//    if( point.pose.position.x > (mStartingPoint[0] + mWidth) || point.pose.position.x < mStartingPoint[0])
//        return false;
//    else if(point.pose.position.y > (mStartingPoint[1] + mLength) || point.pose.position.y < mStartingPoint[1])
//        return false;
//    else if(point.pose.position.z > (mStartingPoint[2] + mHeight) || point.pose.position.z < mStartingPoint[2])
//        return false;

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


void Building::CreatePerimPoints()
{
    LOG_IF(ERROR, mLength == 0 || mWidth ==0 || mHeight ==0)<<"Building parameters are empty!"<<endl;

    LOG(INFO)<<"CreatePerimPoints Start"<<endl;

/*  C2        C3
 *   __________    +x
 *   |        |    ^
 *   |________|    |
 *   C1       C0   |
 *                 |
 *  +y <-----------|0
 */

    float grid_size = 0.3;
    for(float z = mC0[2]; z<=mC0[2]+mHeight; )
    {
        // C1 -> C2
        // C0 -> C3
        for (float x = mC0[0]; x <= mC3[0]; )
        {
            Eigen::Vector3f temp(x, mC0[1], z);
            mPerimPoints.emplace_back(x, mC0[1], z);
            mPerimPoints.emplace_back(x, mC1[1], z);
            x += grid_size;
        }
        // C0 -> C1
        // C2 -> C3
        for (float y = mC0[1]; y <= mC1[1]; )
        {
            mPerimPoints.emplace_back(mC0[0], y, z);
            mPerimPoints.emplace_back(mC3[0], y, z);
            y += grid_size;
        }

        z += grid_size;
    }


}

void Building::CreateDoorPoints()
{

    /*  C2        C3
 *   __________    +x
 *   |        |    ^
 *   |____D___|    |
 *   C1       C0   |
 *                 |
 *  +y <-----------|0
 */

    Eigen::Vector3f door_global_position(mC0[0]+mDoorPosition[0],
                                         mC0[1]+mDoorPosition[1] + mDoorWidth/2.0,
                                         mC0[2]+mDoorPosition[2]);

    mDoorGlobalPosition = door_global_position;

    float grid_size = 0.2;
    for(float w=0; w<=mDoorWidth; )
    {
        Eigen::Vector3f temp_points;
        for(float h=0; h<=mDoorHeight; )
        {
            mDoorPoints.emplace_back(mDoorGlobalPosition[0],
                                     mDoorGlobalPosition[1] + w,
                                     mDoorGlobalPosition[2] + h);
            h += grid_size;
        }
        w += grid_size;
    }
}

void Building::PublishBuildingPoints()
{
    LOG(INFO)<<"PublishBuildingPoints started!"<<endl;

    visualization_msgs::Marker mark;
    for(auto& pt : mPerimPoints)
    {
        ros::Duration(0.001).sleep();
        mark.header.frame_id="/map";
        mark.id = mMarkerStartingID;
        mark.color.a = 1.0;
        mark.color.r = 1.0;
        mark.color.g = 0.0;
        mark.color.b = 0.0;
        mark.pose.position.x = pt[0];
        mark.pose.position.y = pt[1];
        mark.pose.position.z = pt[2];
        mark.pose.orientation.x = 1;
        mark.pose.orientation.y = 0;
        mark.pose.orientation.z = 0;
        mark.pose.orientation.w = 0;
        mark.scale.x = 0.1;
        mark.scale.y = 0.1;
        mark.scale.z = 0.1;
        mark.action = visualization_msgs::Marker::ADD;
        mark.type = visualization_msgs::Marker::CUBE;

        mBuildingPub.publish(mark);
        mMarkerStartingID ++;
    }

    for(auto& pt : mDoorPoints)
    {
        ros::Duration(0.001).sleep();
        mark.header.frame_id="/map";
        mark.id = mDoorMarkerStartingID;
        mark.color.a = 1.0;
        mark.color.r = 0.0;
        mark.color.g = 1.0;
        mark.color.b = 0.0;
        mark.pose.position.x = pt[0];
        mark.pose.position.y = pt[1];
        mark.pose.position.z = pt[2];
        mark.pose.orientation.x = 1;
        mark.pose.orientation.y = 0;
        mark.pose.orientation.z = 0;
        mark.pose.orientation.w = 0;
        mark.scale.x = 0.1;
        mark.scale.y = 0.1;
        mark.scale.z = 0.1;
        mark.action = visualization_msgs::Marker::ADD;
        mark.type = visualization_msgs::Marker::CUBE;

        mBuildingPub.publish(mark);
        mDoorMarkerStartingID ++;
    }

    LOG(INFO)<<"PublishBuildingPoints finished!"<<endl;
}

