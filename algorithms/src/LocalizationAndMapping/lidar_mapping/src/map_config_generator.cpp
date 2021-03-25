#include <cstdio>
#include <glog/logging.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

//#define foreach_boost BOOST_FOREACH

#include "typedefs.h"
#include <opencv2/core/persistence.hpp>

// 通过初始GPS坐标与航姿参考,对整幅地图作坐标系变换,使0，0，0对应初始GPS点,旋转地图使其采用NWU坐标系,保存旋转后的地图文件,并创建配置文件供map_publisher读取.

// 目前仅支持px4飞控的消息. 这个模块与地图读取定位等隔离，因此可以做自己的高精地图实现而不影响其他组件.



bool loadPCDmap(MapCloudT::Ptr& pmap_cloud)
{
    string map_path;
    bool path_exist = ros::param::get("map_path",map_path);
    if(!path_exist)
    {
        LOG(ERROR)<<"Fatal error in map_config_generator: get map_path failed!"<<endl;
        LOG(ERROR)<<"map_path:"<<map_path<<endl;
        exit(-1);
    }
    pmap_cloud = MapCloudT::Ptr(new MapCloudT);
    pcl::io::loadPCDFile(map_path, *pmap_cloud);
    if(pmap_cloud->size()>0)
    {
        LOG(INFO)<<"map pointcloud size:"<<pmap_cloud->size()<<endl;
        return true;
    }
    return false;
}
bool loadFirstOdometryFromROSBagFile(nav_msgs::Odometry& ahrs_msg,sensor_msgs::NavSatFix& gps_msg)//同时读取gps坐标和航姿信息.
{
    string bag_path;
    bool path_exist = ros::param::get("rosbag_path",bag_path);
    if(!path_exist)
    {
        LOG(ERROR)<<"Fatal error in map_config_generator: get bag_path failed!"<<endl;
        LOG(ERROR)<<"bag_path:"<<bag_path<<endl;
        exit(-1);
    }
    rosbag::Bag bag;
    bag.open(bag_path);
    vector<string> topics;
    string ahrs_local_topic = "/mavros/global_position/local";
    string gps_global_topic = "/mavros/global_position/raw/fix";
    topics.push_back(ahrs_local_topic);
    topics.push_back(gps_global_topic);
    rosbag::View views(bag,rosbag::TopicQuery(topics));
    bool flag_ahrs = false;
    bool flag_gps = false;

    ros::Time ahrs_msg_timestamp;
    ros::Time gps_msg_timestamp;

    for(rosbag::MessageInstance const m:views)
    {
        if(m.getTopic()==ahrs_local_topic)
        {
            nav_msgs::Odometry::Ptr p_odom_msg = m.instantiate<nav_msgs::Odometry>();
            ahrs_msg = *p_odom_msg;
            //这两个时间是sim world里的时间,不是bag开始时的时间.
//            if(m.getTime().toSec()>2)
//            {
//                LOG(ERROR)<<"[map_config_generator] ERROR: Odometry msg is not found in initial 2s!"<<endl;
//                throw "Error";
//            }
            flag_ahrs = true;
            ahrs_msg_timestamp = m.getTime();

        }
        else if(m.getTopic()==gps_global_topic)
        {
            sensor_msgs::NavSatFix::Ptr p_gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
            gps_msg = *p_gps_msg;
//            if(m.getTime().toSec()>2)
//            {
//                LOG(ERROR)<<"[map_config_generator] ERROR: GPS msg is not found in initial 2s!"<<endl;
//                throw "Error";
//            }
            flag_gps = true;
            gps_msg_timestamp = m.getTime();
        }
        if(flag_ahrs&&flag_gps)
        {
            double time_diff = (gps_msg_timestamp-ahrs_msg_timestamp).toSec();
            if(time_diff > 0.5||time_diff<-0.5)
            {
                LOG(ERROR)<<"[map_config_generator] ERROR: GPS AHRS time diff >0.5s!"<<endl;
                throw "Error";
            }
            break;
        }
    }
    if(!(flag_ahrs&&flag_gps))
    {
        LOG(ERROR)<< "[map_config_generator] get gps and ahrs msg failed"<<endl;
        throw "Error";
    }
    bag.close();
    return true;
}

void transformMapByInitialOdometryAndGPSCoordinate(const nav_msgs::Odometry& odom_msg,const sensor_msgs::NavSatFix& gps_msg,MapCloudT::Ptr pMap)
{
    LOG(INFO)<<"transforming map..."<<endl;
    string output_map_path;
    ros::param::get("output_map_path",output_map_path);
    string output_map_config_path = output_map_path+".yaml";
    //绕0，0，0旋转地图坐标系，获取新地图并保存.
    //pcl::transformPointCloud(*pcloud_current,*transformed,output_pose);
    MapCloudT::Ptr new_map(new MapCloudT);
    Eigen::Quaterniond quat;

        auto ps = odom_msg.pose.pose.orientation;
        LOG(INFO)<<"Quaternion:"<<ps.x<<";"<<ps.y<<";"<<ps.z<<";"<<ps.w<<endl;
        quat.x() = ps.x;
        quat.y() = ps.y;
        quat.z() = ps.z;
        quat.w() = ps.w;

        Eigen::Matrix3d R_flu_luf;
        R_flu_luf<<1,0,0,0,0,-1,0,1,0;//正确.


        Eigen::Matrix3d rot_original = quat.toRotationMatrix();
        Eigen::Matrix3d rot_transformed_ = R_flu_luf.inverse()*rot_original.inverse()*(R_flu_luf);
        Eigen::Matrix3d rot_LUF = rot_transformed_.inverse();
        Eigen::Quaterniond rot_flu (R_flu_luf*rot_LUF);


    //Eigen::Quaterniond quat_inv = quat;

    //Eigen::Matrix3d rot = quat.toRotationMatrix();
    //const Eigen::Matrix<Scalar, 3, 1> trans(0,0,0);
    const Eigen::Vector3d trans(0,0,0);

    pcl::transformPointCloud(*pMap,*new_map,trans,rot_flu);

    pcl::io::savePCDFile(output_map_path,*new_map);

    //保存新地图的0，0，0所对应的经纬度到配置文件.
    LOG(INFO)<<"saving config file..."<<endl;

    std::FILE* fp_config_output;
    fp_config_output = fopen(output_map_config_path.c_str(),"w");
    fprintf(fp_config_output,"\%YAML:1.0\n");
    fprintf(fp_config_output,"---\n");
    fprintf(fp_config_output,"initial_longitude: %.10f\n",gps_msg.longitude);
    fprintf(fp_config_output,"initial_latitude: %.10f\n",gps_msg.latitude);
    fprintf(fp_config_output,"initial_altitude: %.10f\n",gps_msg.altitude);
    fprintf(fp_config_output,"coordinate_mode: \"NWU\" ");
    fclose(fp_config_output);
    return;
    //    cv::FileStorage fs;
    //    fs.open(output_map_config_path,cv::FileStorage::WRITE);
    //    fs.writeObj();



}

bool run_all_pipeline()
{
    LOG(INFO)<<"run all pipeline..."<<endl;
    MapCloudT::Ptr pMap;
    nav_msgs::Odometry odom_msg;
    sensor_msgs::NavSatFix gps_msg;
    bool flag_map = loadPCDmap(pMap);
    LOG(INFO)<<"map loaded..."<<endl;
    bool flag_bag = loadFirstOdometryFromROSBagFile(odom_msg,gps_msg);
    LOG(INFO)<<"bag loaded..."<<endl;

    if(flag_map&&flag_bag)
    {
        transformMapByInitialOdometryAndGPSCoordinate(odom_msg,gps_msg,pMap);
    }
    return true;

}

int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("map_config_generator_node");
    ros::init(argc,argv,"map_config_generator_node");
    LOG(INFO)<<"Start node map_config_generator"<<endl;
    run_all_pipeline();

    return 0;
}
