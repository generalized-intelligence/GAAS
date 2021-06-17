#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_dense_reconstruction/CamToRobotCalibParamsConfig.h>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "popt_pp.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include "pcl_helper.h"

using namespace cv;
using namespace std;

ros::Publisher output_cloud_pub;

void sync_pc_callback(sensor_msgs::PointCloud2::ConstPtr ppc,geometry_msgs::PoseStamped::ConstPtr ppose)
{
    std::cout<<"Point Clound Sync callback"<<std::endl;
    
    sensor_msgs::PointCloud2 pc2 = * ppc;

    sensor_msgs::PointCloud pc;
    bool result = sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);

    geometry_msgs::PoseStamped pose = *ppose;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
    Eigen::Matrix3d mat = q.toRotationMatrix();

    for (int i = 0;i<3;i++)
    {
        for(int j =0 ;j<3;j++)
        {
            transform(i,j) = mat(i,j);
            std::cout<<transform(i,j)<<"	|	";
        }
        std::cout <<std::endl;
    }
    

    transform(0,3) = pose.pose.position.x;
    transform(1,3) = pose.pose.position.y;
    transform(2,3) = pose.pose.position.z;


    for(int k=0; k<3; k++)
    {
      std::cout<<transform(k,3)<<" |";
    }
    std::cout<<"ENDL"<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto iter = pc.points.begin();iter!=pc.points.end();++iter)
    {
        pcl::PointXYZ p;
        p.x = (*iter).x;
        p.y = (*iter).y;
        p.z = (*iter).z;
        original_cloud->push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*original_cloud,*transformed_cloud,transform);
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
    pcl::toROSMsg(*transformed_cloud,pc_out_pointcloud2);
    pc_out_pointcloud2.header.frame_id = "map";
    output_cloud_pub.publish(pc_out_pointcloud2);
}

int main(int argc,char** argv)
{
    string node_name = "pointcloud_to_global_frame";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    output_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in",1);

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud(nh,"/camera/left/point_cloud2", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(nh,"/mavros/local_position/pose", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), sub_pointcloud, sub_pose);
    
    sync.registerCallback(boost::bind(&sync_pc_callback, _1, _2));
    
    ros::spin();
    return 0;
}


