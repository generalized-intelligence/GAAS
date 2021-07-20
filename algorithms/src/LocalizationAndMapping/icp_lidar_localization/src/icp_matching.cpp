#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Geometry>


#include "Timer.h"
#include "icp_algo.h"
#include "GPS_AHRS_sync.h"

ICPAlgo* pICP;
tf2_ros::TransformBroadcaster* pTFbroadcaster;
bool icp_result_visualization = false;
ros::Publisher pose_pub,gps_ahrs_pose_pub;
ros::Publisher aligned_cloud_pub;
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ScopeTimer timer("callback_timer");
    if(pICP->initializeICPPoseGuess())//测试是否能获取gps-ahrs消息.
    {

        Eigen::Matrix4f gps_ahrs_initial_guess = pICP->gps_ahrs_initial_guess;
        Eigen::Matrix3f rotmat = gps_ahrs_initial_guess.block(0,0,3,3);
        Eigen::Quaternionf quat(rotmat);
        geometry_msgs::PoseStamped gps_ahrs_pose_msg;

        gps_ahrs_pose_msg.header.frame_id="map";//地图坐标系的定位.
        gps_ahrs_pose_msg.header.stamp = cloud_msg->header.stamp;//与gps同步.

        auto& pos = gps_ahrs_pose_msg.pose.position;
        auto& orient = gps_ahrs_pose_msg.pose.orientation;
        orient.x = quat.x();
        orient.y = quat.y();
        orient.z = quat.z();
        orient.w = quat.w();

        pos.x = gps_ahrs_initial_guess(0,3);
        pos.y = gps_ahrs_initial_guess(1,3);
        pos.z = gps_ahrs_initial_guess(2,3);

        gps_ahrs_pose_pub.publish(gps_ahrs_pose_msg);
        LOG(INFO)<<"GPS AHRS pose published!"<<endl;

    }
    cout <<"callback"<<endl;
    LOG(INFO)<<"In lidar callback:"<<endl;
    LidarCloudT::Ptr pInputCloud(new LidarCloudT);
    pcl::fromROSMsg(*cloud_msg, *pInputCloud);
    LOG(INFO)<<"In lidar callback: lidar pointcloud size:"<<pInputCloud->size()<<endl;
    //Downsampling.
    LidarCloudT::Ptr downsampled(new LidarCloudT);
    pcl::VoxelGrid<LidarPointT> sor;
    sor.setInputCloud(pInputCloud);
    //sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.setLeafSize(DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE);
    sor.filter(*downsampled);
    timer.watch("till cloud downsampled");
    LOG(INFO)<<"Downsampled pointcloud size:"<<downsampled->size()<<endl;
    Eigen::Matrix4f output_icp_pose;

    bool need_transformed_pointcloud = (icp_result_visualization&&aligned_cloud_pub.getNumSubscribers()>0);
    LidarCloudT::Ptr transformed_cloud=nullptr;
    bool icp_success = pICP->doICPMatching(downsampled,output_icp_pose,transformed_cloud,
                                           need_transformed_pointcloud,cloud_msg->header.stamp);
    timer.watch("till icp finished:");


    if(icp_success)
    {
        //publish output pose;
        Eigen::Matrix3f rotmat = output_icp_pose.block(0,0,3,3);
        Eigen::Quaternionf quat(rotmat);
        geometry_msgs::PoseStamped icp_pose_msg;

        icp_pose_msg.header.frame_id="map";//地图坐标系的定位.
        icp_pose_msg.header.stamp = cloud_msg->header.stamp;//与雷达同步.

        auto& pos = icp_pose_msg.pose.position;
        auto& orient = icp_pose_msg.pose.orientation;
        orient.x = quat.x();
        orient.y = quat.y();
        orient.z = quat.z();
        orient.w = quat.w();

        pos.x = output_icp_pose(0,3);
        pos.y = output_icp_pose(1,3);
        pos.z = output_icp_pose(2,3);

        timer.watch("before publishing pose:");
        pose_pub.publish(icp_pose_msg);

        timer.watch("till pose published:");

        //publish tf2 transfrom.
        Eigen::Matrix4f lidar_to_map = output_icp_pose.inverse();
        Eigen::Matrix3f l_m_rotmat = lidar_to_map.block(0,0,3,3);
        Eigen::Quaternionf l_m_quat(l_m_rotmat);



        geometry_msgs::TransformStamped map_lidar_trans_stamped;
        map_lidar_trans_stamped.header.frame_id = "lidar";// we've got lidar as the top node of tf tree.
        //So when localization is not avail, still we can solve lidar to body.
        map_lidar_trans_stamped.child_frame_id = "map";

        map_lidar_trans_stamped.header.stamp = cloud_msg->header.stamp;
        map_lidar_trans_stamped.transform.translation.x = lidar_to_map(0,3);
        map_lidar_trans_stamped.transform.translation.y = lidar_to_map(1,3);
        map_lidar_trans_stamped.transform.translation.z = lidar_to_map(2,3);
        map_lidar_trans_stamped.transform.rotation.x = l_m_quat.x();
        map_lidar_trans_stamped.transform.rotation.y = l_m_quat.y();
        map_lidar_trans_stamped.transform.rotation.z = l_m_quat.z();
        map_lidar_trans_stamped.transform.rotation.w = l_m_quat.w();

        pTFbroadcaster->sendTransform(map_lidar_trans_stamped);


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
        LOG(WARNING)<<"ICP matching failed!"<<endl;
    }
}

GPS_AHRS_Synchronizer gps_ahrs_sync;


void gps_ahrs_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom)// 同步gps和ahrs.
{
    LOG(INFO)<<"In gps_ahrs_callback"<<endl;
    gps_ahrs_sync.sync_mutex.lock();
    gps_ahrs_sync.gps_msg = *gps_msg;
    gps_ahrs_sync.ahrs_msg = *odom;
    gps_ahrs_sync.ever_init = true;
    gps_ahrs_sync.sync_mutex.unlock();
};

int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("icp_matching_node");
    ros::init(argc,argv,"icp_matching_node");
    ros::NodeHandle nh;
    ICPAlgo icp(&gps_ahrs_sync);
    icp.loadPCDMap();
    pICP=&icp;
    tf2_ros::TransformBroadcaster br;
    pTFbroadcaster = &br;


    string lidar_topic_name;
    ros::param::get("lidar_topic_name",lidar_topic_name);
    ros::param::get("icp_result_visualization",icp_result_visualization);
    LOG(INFO)<<"Subscribing lidar topic name:"<<lidar_topic_name<<endl;
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gaas/localization/registration_pose",1);
    gps_ahrs_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gaas/localization/original_gps_ahrs_pose",1);
    if(icp_result_visualization)
    {
        aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/gaas/visualization/localization/icp_merged_cloud",1);
    }
    ros::Subscriber lidar_sub = nh.subscribe("/gaas/preprocessing/velodyne_downsampled",1,lidar_callback);

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/mavros/global_position/raw/fix", 1);
    //message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/mavros/global_position/global", 1);
    //message_filters::Subscriber<nav_msgs::Odometry> ahrs_sub(nh, "/mavros/local_position/odom", 1);
    message_filters::Subscriber<nav_msgs::Odometry> ahrs_sub(nh, "/mavros/global_position/local", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), gps_sub, ahrs_sub);
    sync.registerCallback(boost::bind(&gps_ahrs_callback, _1, _2));
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
