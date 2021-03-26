#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>

#include "prev_fusion.h"

PrevFusion* pFusion=nullptr;
void LidarImageCallbackFunction(const sensor_msgs::ImageConstPtr& img_msg,const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::cout<<"In LidarImageCallbackFunction():Enter callback function."<<std::endl;
    LidarCloudT::Ptr pInputCloud(new LidarCloudT);
    pcl::fromROSMsg(*cloud_msg, *pInputCloud);
    LidarCloudT::Ptr downsampled_input(new LidarCloudT);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(pInputCloud);
    sor.setLeafSize(0.2, 0.2, 0.2);
    sor.filter(*downsampled_input);



    cv_bridge::CvImagePtr image_ptr;
    try
    {
        image_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e)
    {
        LOG(ERROR)<<"cv_bridge exception: "<< e.what()<<endl;
        return;
    }
    cv::Mat img = image_ptr->image;
    if(pFusion == nullptr)
    {
        LOG(ERROR)<<"Fusion manager is null."<<endl;
        exit(-1);
    }
    vector<cv::Point2f> p2ds;
    bool visualize;
    ros::param::get("/vision_lidar_fusion_node/visualize",visualize);
    pFusion->projectLidarToRGBImageForVisualization(img,*downsampled_input,p2ds,visualize);
}
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"Start vision_lidar_fusion_node."<<endl;
    ros::init(argc,argv,"vision_lidar_fusion_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/gi/simulation/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/gaas/preprocessing/velodyne_downsampled", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, lidar_sub);
    sync.registerCallback(boost::bind(&LidarImageCallbackFunction, _1, _2));


    string config_file_path;
    ros::param::get("/vision_lidar_fusion_node/config_file_path",config_file_path);
    pFusion = new PrevFusion(config_file_path);

    ros::spin();
    return 0;
}



