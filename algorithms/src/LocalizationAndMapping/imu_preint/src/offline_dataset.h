#ifndef OFFLINE_DATASET_H_FILE
#define OFFLINE_DATASET_H_FILE

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

using std::vector;
using std::map;
using std::string;
using std::cout;
using std::endl;



template <typename T>
bool compareHeaderTimeStamp(const T& msg1,const T&msg2)
{
    if(msg1.header.stamp.toNSec()<msg2.header.stamp.toNSec())
    {
        return true;
    }
    return false;
}
struct SequenceDataset
{
    vector<sensor_msgs::Imu> imu_sequence;
    vector<geometry_msgs::PoseStamped> pose_sequence;
    void sortDataset()
    {
        std::sort_heap(imu_sequence.begin(),imu_sequence.end(),compareHeaderTimeStamp<sensor_msgs::Imu>);
        std::sort_heap(pose_sequence.begin(),pose_sequence.end(),compareHeaderTimeStamp<geometry_msgs::PoseStamped>);
    }
    int imu_index = 0;
    int pose_index = 0;
    bool next(vector<sensor_msgs::Imu>& imu_vec,geometry_msgs::PoseStamped& pose)
    {
        imu_vec.clear();
        if(pose_index<pose_sequence.size()-1)
        {
            pose = pose_sequence.at(pose_index);
            pose_index++;
        }
        else
        {
            return false;
        }
        auto next_pose = pose_sequence.at(pose_index);
        for(int i = imu_index;i<imu_sequence.size();i++)
        {
            sensor_msgs::Imu& imu_msg = imu_sequence.at(i);
            if(imu_msg.header.stamp.toNSec()<next_pose.header.stamp.toNSec())
            {
                imu_vec.push_back(imu_msg);
            }
            else
            {
                imu_index = i;
                break;
            }
            //cout<<" dt:"<<(imu_vec.back().header.stamp-imu_vec.front().header.stamp).toSec()<<endl;
        }
        return true;
    }

};


void loadDatasetFromBag(SequenceDataset& sd)
{
    rosbag::Bag bag;
    bag.open("/home/gi/Downloads/imu_localization.bag");
    vector<string> topics;
    topics.push_back(std::string("/gaas/localization/registration_pose"));
    topics.push_back(std::string("/external_imu"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance m: view)
    {
        sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        if (imu != NULL)
        {
            sd.imu_sequence.push_back(*imu);
        }
        geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
        if (pose != NULL)
        {
            sd.pose_sequence.push_back(*pose);
        }
    }
    bag.close();
    sd.sortDataset();
}
#endif
