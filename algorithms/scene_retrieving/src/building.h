//
// Created by gishr on 2019/10/10.
//

#ifndef SCENE_RETRIEVING_BUILDING_H
#define SCENE_RETRIEVING_BUILDING_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <glog/logging.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class Building {

public:

    Building(Eigen::Vector3f& point, int starting_id, Eigen::Vector3f& door_position, int door_starting_id,
             float door_w = 2, float door_h=3, float length=10, float width=5, float height=8);

    template<class T>
    inline bool isInBuilding(T& point)
    {
        if( point[0] > (mStartingPoint[0] + mWidth) || point[0] < mStartingPoint[0])
            return false;
        else if(point[1] > (mStartingPoint[1] + mLength) || point[1] < mStartingPoint[1])
            return false;
        else if(point[2] > (mStartingPoint[2] + mHeight) || point[2] < mStartingPoint[2])
            return false;
        else
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

    inline Eigen::Vector3f DoorGlobalPosition()
    {
        return mDoorGlobalPosition;
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
};


#endif //SCENE_RETRIEVING_BUILDING_H
