#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>        
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;

ros::Publisher output_cloud_pub;
sensor_msgs::PointCloud cur_pointcloud;

void PC_callback(const sensor_msgs::PointCloud2& temp_pc)
{
    sensor_msgs::PointCloud pc;
    bool result = sensor_msgs::convertPointCloud2ToPointCloud(temp_pc, pc);
    cur_pointcloud = pc;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0,0) = 0;
    transform(0,2) = 1;
    transform(1,1) = 0;
    transform(1,0) = -1;
    transform(2,2) = 0;
    transform(2,1) = -1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto iter = cur_pointcloud.points.begin(); iter!=cur_pointcloud.points.end(); ++iter)
    {
        pcl::PointXYZ p;
        p.x = (*iter).x;
        p.y = (*iter).y;
        p.z = (*iter).z;
        original_cloud->push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*original_cloud, *transformed_cloud, transform);
    sensor_msgs::PointCloud  pc_out;
    for(auto iter = original_cloud->points.begin(); iter!=original_cloud->points.end(); ++iter)
    {
        geometry_msgs::Point32 p;
        p.x=(*iter).x;
        p.y=(*iter).y;
        p.z=(*iter).z;
        pc_out.points.push_back(p);
    }
    sensor_msgs::PointCloud2 pc_out_pointcloud2;
    pcl::toROSMsg(*transformed_cloud, pc_out_pointcloud2);
    pc_out_pointcloud2.header.frame_id = "map";
    output_cloud_pub.publish(pc_out_pointcloud2);
}

int main(int argc,char** argv)
{
    string node_name = "transform_cam_to_world";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;


    cout<<"Node initialized as: "<<node_name<<endl;

    output_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in",1);

    ros::Subscriber pc_sub = nh.subscribe("/gi/rgb/depth/points", 1, PC_callback);

    cout<<"sync_pc_callback 1"<<endl;

    ros::spin();
    return 0;
}