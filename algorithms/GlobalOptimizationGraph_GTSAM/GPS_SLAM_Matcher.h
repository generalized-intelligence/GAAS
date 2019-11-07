#ifndef GPS_SLAM_MATCHER_H
#define GPS_SLAM_MATCHER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <glog/logging.h>

#include <opencv2/core/persistence.hpp>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "GPSExpand.h"
#include "CallbacksBufferBlock.h"
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;



double calc_gps_covariance(const sensor_msgs::NavSatFix& gps)
{
    return sqrt(pow(gps.position_covariance[0],2) + pow(gps.position_covariance[4],2));
}

struct slam_gps_match
{
    int slam_index;
    int gps_index;
    slam_gps_match(int slam_i,int gps_i)
    {
        slam_index = slam_i;
        gps_index = gps_i;
    }
};


Vector3d getVec3dPosFromSLAMMsg(const geometry_msgs::PoseStamped& msg)
{//这里不再做任何矩阵变换，因为在callback里已经对buffer中的消息进行过修正。
    auto _p = msg.pose.position;
    Vector3d vec(_p.x,_p.y,_p.z);
    return vec;
}



double euc_dist(double x,double y)
{
    return sqrt(x*x+y*y);
}

class GPS_SLAM_MATCHER
{

public:

    GPS_SLAM_MATCHER(CallbackBufferBlock<geometry_msgs::PoseStamped>* pslam_buffer,CallbackBufferBlock<sensor_msgs::NavSatFix>* pgps_pos_buffer,cv::FileStorage* pSettings)
    {
        this->pslam_buf = pslam_buffer;
        this->pgps_pos_buf = pgps_pos_buffer;
        this->pSettings = pSettings;
    }

    inline int matchLen()
    {
        return match_vec.size();
    }

    slam_gps_match queryMatchByIndex(int index)
    {
        return this->match_vec[index];
    }

    // add one pair of matched index
    bool addMatch(int slam_index, int gps_index)
    {
        double slam_gps_delay_thres = (*pSettings)["SLAM_GPS_DELAY_THRESHOLD"];
        double gps_pos_thres_m = (*pSettings)["GPS_POS_THRES_m"];

        if (calc_gps_covariance(pgps_pos_buf->at(gps_index) )< gps_pos_thres_m)
        {
            this->match_vec.emplace_back(slam_index, gps_index);
            return true;
        }

        if(!abs(pslam_buf->at(slam_index).header.stamp.toSec()-pgps_pos_buf->at(gps_index).header.stamp.toSec() ) < slam_gps_delay_thres)
        {
            LOG(WARNING)<<"gps slam delay >thres!Delay:"<<abs(pslam_buf->at(slam_index).header.stamp.toSec() - pgps_pos_buf->at(gps_index).header.stamp.toSec() )<<endl;
        }

        if(calc_gps_covariance(pgps_pos_buf->at(gps_index) )>= gps_pos_thres_m)
        {
            LOG(WARNING)<<"gps variance > thres.variance:"<< calc_gps_covariance(pgps_pos_buf->at(gps_index) )<<endl;
        }

        return false;
    }

    float findAverage(vector<float> ValueVec)
    {
        if(ValueVec.empty())
            return 0;

        float sum;
        for(auto& yaw : ValueVec)
        {
            sum += yaw;
        }

        return sum / ValueVec.size();
    }

    void check2IndexAndCalcDeltaDegInit(int matchindex1, int matchindex2, GPSExpand& GPS_coord, bool& valid, double& delta_deg, double& deg_variance)
    {
        LOG(INFO)<<"In check2IndexAndCalcDeltaDeg(): matchindex:"<<matchindex1<<","<<matchindex2<<";"<<endl;
        LOG(INFO)<<"		GPS index:"<<match_vec[matchindex1].gps_index<<","<<match_vec[matchindex2].gps_index<<";"<<endl;
        LOG(INFO)<<"		SLAM index:"<<match_vec[matchindex1].slam_index<<","<<match_vec[matchindex2].slam_index<<";"<<endl;

        double p1_variance = calc_gps_covariance(pgps_pos_buf->at(match_vec[matchindex1].gps_index));
        double p2_variance = calc_gps_covariance(pgps_pos_buf->at(match_vec[matchindex2].gps_index));

        //calc_gps_covariance(matchindex1.gps_index,....)
        valid = false;
        deg_variance = -1;

        Vector3d slam_pos0 = getVec3dPosFromSLAMMsg(pslam_buf->at(match_vec[matchindex1].slam_index));
        Vector3d slam_pos1 = getVec3dPosFromSLAMMsg(pslam_buf->at(match_vec[matchindex2].slam_index));
        Vector3d slam_diff = slam_pos1 - slam_pos0;
        double delta_lon = pgps_pos_buf->at(match_vec[matchindex2].gps_index).longitude - pgps_pos_buf->at(match_vec[matchindex1].gps_index).longitude;
        double delta_lat = pgps_pos_buf->at(match_vec[matchindex2].gps_index).latitude - pgps_pos_buf->at(match_vec[matchindex1].gps_index).latitude;
        double gps_dx = delta_lon*GPS_coord.vari_km_per_lon_deg()*1000;
        double gps_dy = delta_lat*GPS_coord.vari_km_per_lat_deg()*1000;
        double slam_dx = slam_diff[0];
        double slam_dy = slam_diff[1];

        double theta_gps = atan2(gps_dy, gps_dx);
        double theta_slam = atan2(slam_dy, slam_dx);
        double len_gps = euc_dist(gps_dx, gps_dy);
        double len_slam = euc_dist(slam_dx, slam_dy);
        double yaw_deg = 180*(theta_slam - theta_gps)/3.1415926535;

        LOG(INFO)<<"slam_pos1 x y: "<<slam_pos1[0]<<", "<<slam_pos1[1]<<endl;
        LOG(INFO)<<"SLAM dx dy in m: "<<slam_dx<<", "<<slam_dy<<endl;
        LOG(INFO)<<"GPS dx dy in m: "<<gps_dx<<", "<<gps_dy<<endl;
        LOG(INFO)<<"SLAM arcTan: "<<theta_slam<<", "<<180*theta_slam/3.1415926535<<endl;
        LOG(INFO)<<"gps arcTan: "<<theta_gps<<", "<<180*theta_gps/3.1415926535<<endl;
        LOG(INFO)<<"yaw_deg from slam to gps: "<<yaw_deg<<endl;
        LOG(INFO)<<"mYawOffsetVec.size(): "<<mYawOffsetVec.size()<<endl;
        //vector<float> mYawOffsetVec;
        //float mFinalYawOffset = -1;

        if(mYawOffsetVec.empty())
        {
            mYawOffsetVec.push_back(yaw_deg);
            valid = false;
            return;
        }

        float average_yaw = findAverage(mYawOffsetVec);
        //TODO find a better way
        if(abs(average_yaw - yaw_deg) < 15)
            mYawOffsetVec.push_back(yaw_deg);

        if(len_gps<1.2*len_slam && len_gps>0.8*len_slam && len_gps> 10)
        {
            LOG(INFO)<<"GPS_SLAM_YAW_CALCED:"<<yaw_deg<<endl;
            //delta_deg = yaw_deg;
            delta_deg = findAverage(mYawOffsetVec);
            LOG(INFO)<<"delta_deg: "<<delta_deg<<endl;
            deg_variance = 180*((p1_variance+p2_variance)/len_gps)/3.1415926535;
            valid = true;
        }
        else
        {
            LOG(INFO)<<"NOT AVAIL:YAW_VALUE_FOR_CHECK:"<< yaw_deg<<"len_gps:"<<len_gps<<endl;
        }
    }

    void check2IndexAndCalcDeltaDeg(int matchindex1, int matchindex2, GPSExpand& GPS_coord, bool& valid, double& delta_deg, double& deg_variance)
    {
        LOG(INFO)<<"In check2IndexAndCalcDeltaDeg(): matchindex:"<<matchindex1<<","<<matchindex2<<";"<<endl;
        LOG(INFO)<<"		GPS index:"<<match_vec[matchindex1].gps_index<<","<<match_vec[matchindex2].gps_index<<";"<<endl;
        LOG(INFO)<<"		SLAM index:"<<match_vec[matchindex1].slam_index<<","<<match_vec[matchindex2].slam_index<<";"<<endl;

        double p1_variance = calc_gps_covariance(pgps_pos_buf->at(match_vec[matchindex1].gps_index));
        double p2_variance = calc_gps_covariance(pgps_pos_buf->at(match_vec[matchindex2].gps_index));

        //calc_gps_covariance(matchindex1.gps_index,....)
        valid = false;
        deg_variance = -1;

        Vector3d slam_pos0 = getVec3dPosFromSLAMMsg(pslam_buf->at(match_vec[matchindex1].slam_index));
        Vector3d slam_pos1 = getVec3dPosFromSLAMMsg(pslam_buf->at(match_vec[matchindex2].slam_index));
        Vector3d slam_diff = slam_pos1 - slam_pos0;
        double delta_lon = pgps_pos_buf->at(match_vec[matchindex2].gps_index).longitude - pgps_pos_buf->at(match_vec[matchindex1].gps_index).longitude;
        double delta_lat = pgps_pos_buf->at(match_vec[matchindex2].gps_index).latitude - pgps_pos_buf->at(match_vec[matchindex1].gps_index).latitude;
        double gps_dx = delta_lon*GPS_coord.vari_km_per_lon_deg()*1000;
        double gps_dy = delta_lat*GPS_coord.vari_km_per_lat_deg()*1000;
        double slam_dx = slam_diff[0];
        double slam_dy = slam_diff[1];

        double theta_gps = atan2(gps_dy, gps_dx);
        double theta_slam = atan2(slam_dy, slam_dx);
        double len_gps = euc_dist(gps_dx, gps_dy);
        double len_slam = euc_dist(slam_dx, slam_dy);
        double yaw_deg = 180*(theta_slam - theta_gps)/3.1415926535;

        LOG(INFO)<<"slam_pos1 x y: "<<slam_pos1[0]<<", "<<slam_pos1[1]<<endl;
        LOG(INFO)<<"SLAM dx dy in m: "<<slam_dx<<", "<<slam_dy<<endl;
        LOG(INFO)<<"GPS dx dy in m: "<<gps_dx<<", "<<gps_dy<<endl;
        LOG(INFO)<<"SLAM arcTan: "<<theta_slam<<", "<<180*theta_slam/3.1415926535<<endl;
        LOG(INFO)<<"gps arcTan: "<<theta_gps<<", "<<180*theta_gps/3.1415926535<<endl;
        LOG(INFO)<<"yaw_deg from slam to gps: "<<yaw_deg<<endl;
        LOG(INFO)<<"mYawOffsetVec.size(): "<<mYawOffsetVec.size()<<endl;
        //vector<float> mYawOffsetVec;
        //float mFinalYawOffset = -1;

        float average_yaw = findAverage(mYawOffsetVec);
        //TODO find a better way
        if(abs(average_yaw - yaw_deg) < 15)
            mYawOffsetVec.push_back(yaw_deg);

        if(len_gps<1.2*len_slam && len_gps>0.8*len_slam && len_gps> 10)
        {
            LOG(INFO)<<"GPS_SLAM_YAW_CALCED:"<<yaw_deg<<endl;
            delta_deg = yaw_deg;
            //delta_deg = findAverage(mYawOffsetVec);
            LOG(INFO)<<"delta_deg: "<<delta_deg<<endl;
            deg_variance = 180*((p1_variance+p2_variance)/len_gps)/3.1415926535;
            valid = true;
        }
        else
        {
            LOG(INFO)<<"NOT AVAIL:YAW_VALUE_FOR_CHECK:"<< yaw_deg<<"len_gps:"<<len_gps<<endl;
        }
    }

    slam_gps_match at(int index)
    {
        if(index>=this->match_vec.size())
        {
            LOG(ERROR)<<"In GPS_SLAM_MATCHER at():index overflow!"<<"match_vec.size():"<<match_vec.size()<<",input index:"<<index<<endl;
        }
        return this->match_vec.at(index);
    }

private:

    vector<slam_gps_match> match_vec;
    CallbackBufferBlock<geometry_msgs::PoseStamped>* pslam_buf;
    CallbackBufferBlock<sensor_msgs::NavSatFix>* pgps_pos_buf;
    cv::FileStorage* pSettings;

    vector<float> mYawOffsetVec;
    float mFinalYawOffset = -1;
};


#endif
