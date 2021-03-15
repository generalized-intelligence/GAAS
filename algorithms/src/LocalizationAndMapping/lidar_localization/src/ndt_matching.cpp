#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Geometry>

#include "Timer.h"
#include "ndt_algo.h"

NDTAlgo* pNDT;
bool ndt_result_visualization = false;
ros::Publisher pose_pub;
ros::Publisher aligned_cloud_pub;
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ScopeTimer timer("callback_timer");
    cout <<"callback"<<endl;
    LOG(INFO)<<"In lidar callback:"<<endl;
    LidarCloudT::Ptr pInputCloud(new LidarCloudT);
    pcl::fromROSMsg(*cloud_msg, *pInputCloud);
    LOG(INFO)<<"In lidar callback: lidar pointcloud size:"<<pInputCloud->size()<<endl;
    //Downsampling.
    LidarCloudT::Ptr downsampled(new LidarCloudT);
    pcl::VoxelGrid<LidarPointT> sor;
    sor.setInputCloud(pInputCloud);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(*downsampled);
    timer.watch("till cloud downsampled");
    LOG(INFO)<<"Downsampled pointcloud size:"<<downsampled->size()<<endl;
    NDTAlgo::RTMatrix4f output_ndt_pose;

    bool need_transformed_pointcloud = (ndt_result_visualization&&aligned_cloud_pub.getNumSubscribers()>0);
    LidarCloudT::Ptr transformed_cloud=nullptr;
    bool ndt_success = pNDT->do_ndt_matching_without_initial_guess2(downsampled,output_ndt_pose,transformed_cloud,need_transformed_pointcloud);

    if(ndt_success)
    {
        //publish output pose;
        Eigen::Matrix3f rotmat = output_ndt_pose.block(0,0,3,3);
        Eigen::Quaternionf quat(rotmat);
        geometry_msgs::Pose ndt_pose_msg;
        auto& pos = ndt_pose_msg.position;
        auto& orient = ndt_pose_msg.orientation;
        orient.x = quat.x();
        orient.y = quat.y();
        orient.z = quat.z();

        pos.x = output_ndt_pose(0,3);
        pos.y = output_ndt_pose(1,3);
        pos.z = output_ndt_pose(2,3);

        pose_pub.publish(ndt_pose_msg);
        if(need_transformed_pointcloud&&transformed_cloud!=nullptr)
        {
            sensor_msgs::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*transformed_cloud,transformed_cloud_msg);
            transformed_cloud_msg.header.frame_id = "map";
            transformed_cloud_msg.header.stamp = cloud_msg->header.stamp;
            aligned_cloud_pub.publish(transformed_cloud_msg);
        }
        //publish merged pointcloud
    }
    else
    {
        LOG(WARNING)<<"NDT matching failed!"<<endl;
    }
}


int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("ndt_matching_node");
    ros::init(argc,argv,"ndt_matching_node");
    ros::NodeHandle nh;
    NDTAlgo ndt;
    ndt.loadPCDMap();
    pNDT=&ndt;

    string lidar_topic_name;
    ros::param::get("lidar_topic_name",lidar_topic_name);
    ros::param::get("ndt_result_visualization",ndt_result_visualization);
    LOG(INFO)<<"Subscribing lidar topic name:"<<lidar_topic_name<<endl;
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gaas/localization/ndt_pose",5);
    if(ndt_result_visualization)
    {
        aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("gaas/visualization/localization/ndt_merged_cloud",1);
    }
    ros::Subscriber lidar_sub = nh.subscribe("/velodyne_points2",1,lidar_callback);
    ros::spin();
    return 0;
    //registerCallback(lidar_callback);
    //load map.
    // in callback func:
    // get lidar pointcloud msgs.
    // do ndt matching
    // check matching result.
}
