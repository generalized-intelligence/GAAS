//
// Created by gishr on 2019/10/10.
//

#ifndef SCENE_RETRIEVING_BUILDING_H
#define SCENE_RETRIEVING_BUILDING_H

#include <iostream>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <glog/logging.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;

class Building {

public:

    Building(float height = 5);

    void BuildingPointCallback(const geometry_msgs::PoseStamped& pose);


    inline bool isInBuilding(geometry_msgs::PoseStamped& point, bool use_log=false)
    {
        if(use_log)
        {
            cout << "point.pose.position: " << point.pose.position << endl;
            cout << "(mStartingPoint[0]: " << mStartingPoint[0] << endl;
            cout << "point.pose.position.x > (mStartingPoint[0] + mWidth): "
                 << (point.pose.position.x > (mStartingPoint[0] + mWidth)) << endl;
            cout << "point.pose.position.x < mStartingPoint[0]: " << (point.pose.position.x < mStartingPoint[0])
                 << endl;
            cout << "point.pose.position.y > (mStartingPoint[1] + mLength: "
                 << (point.pose.position.y > (mStartingPoint[1] + mLength)) << endl;
            cout << "point.pose.position.y < mStartingPoint[1]: " << (point.pose.position.y < mStartingPoint[1])
                 << endl;
            cout << "point.pose.position.z > (mStartingPoint[2] + mHeight: "
                 << (point.pose.position.z > (mStartingPoint[2] + mHeight)) << endl;
            cout << "point.pose.position.z < mStartingPoint[2]: " << (point.pose.position.z < mStartingPoint[2])
                 << endl;
        }

        //    /*  C2        C3
        //     *   __________    +x
        //     *   |        |    ^
        //     *   |____D___|    |
        //     *   C1       C0   |
        //     *                 |
        //     *  +y <-----------|0
        //     */

        if( point.pose.position.x > (mCorner_0.pose.position.x + mWidth) || point.pose.position.x < mCorner_0.pose.position.x)
            return false;
        else if(point.pose.position.y > (mCorner_0.pose.position.y + mLength) || point.pose.position.y < mCorner_0.pose.position.y)
            return false;
        else if(point.pose.position.z > (mCorner_0.pose.position.z + mHeight) || point.pose.position.z < mCorner_0.pose.position.z )
            return false;

        return true;
    }

    void CreatePerimPoints();

    void CreateDoorPoints();

    void PublishBuildingPoints();

    template<class T>
    inline float distance(T& p1, T& p2)
    {
        float square_x = (p1[0] - p2[0]) * (p1[0] - p2[0]);
        float square_y = (p1[1] - p2[1]) * (p1[1] - p2[1]);
        float square_z = (p1[2] - p2[1]) * (p1[2] - p2[2]);

        return sqrt(square_x + square_y + square_z);
    }

    inline int PointsNum()
    {
        return mPerimPoints.size();
    }

    inline int DoorPtsNum()
    {
        return mDoorPoints.size();
    }

    inline geometry_msgs::PoseStamped DoorGlobalPosition()
    {
        return mGate;
    }

    void SetCornerPointsAndGate()
    {
        LOG(INFO)<<"Start setting building corner points and gate position, please use RVIZ to select!"<<endl;
        while(mCornerAndGate.size() != 5)
        {
            LOG(INFO)<<"Points received: "<<mCornerAndGate.size()<<"/5"<<endl;
            ros::Duration(2.0).sleep();
        }

        if(mCornerAndGate.size() == 5)
        {
            mCorner_0 = mCornerAndGate[0];
            mCorner_1 = mCornerAndGate[1];
            mCorner_2 = mCornerAndGate[2];
            mCorner_3 = mCornerAndGate[3];
            mGate = mCornerAndGate[4];

            mFinished = true;
            LOG(INFO)<<"Setting finished!!, mGate: "<<mGate<<endl;
        } else{
            LOG(ERROR)<<"Setting failed!, mCornerAndGate size is not 5!"<<endl;
        }

    }

public:

    int mMarkerStartingID = -1;
    int mDoorMarkerStartingID = -1;

    Eigen::Vector3f mStartingPoint;
    Eigen::Vector3f mC0, mC1, mC2, mC3;
    float mLength = 0;
    float mWidth = 0;
    float mHeight = 0;

    vector<Eigen::Vector3f> mPerimPoints;

    Eigen::Vector3f mDoorPosition;
    Eigen::Vector3f mDoorGlobalPosition;
    float mDoorWidth, mDoorHeight;
    vector<Eigen::Vector3f> mDoorPoints;

private:

    ros::NodeHandle mNH;
    ros::Publisher mBuildingPub;

    size_t mMarker_Index = 0;

    vector<Eigen::Vector3f> mCornerPts;

    ros::Subscriber mBuildingSub;

    geometry_msgs::PoseStamped mCorner_0;
    geometry_msgs::PoseStamped mCorner_1;
    geometry_msgs::PoseStamped mCorner_2;
    geometry_msgs::PoseStamped mCorner_3;
    geometry_msgs::PoseStamped mGate;
    vector<geometry_msgs::PoseStamped> mCornerAndGate;

    shared_ptr<std::mutex> mCornerMutex = nullptr;
    bool mFinished = false;
};


#endif //SCENE_RETRIEVING_BUILDING_H
