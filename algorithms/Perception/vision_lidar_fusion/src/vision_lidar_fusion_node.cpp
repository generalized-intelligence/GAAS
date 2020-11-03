#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "prev_fusion.h"

void LidarImageCallbackFunction(const sensor_msgs::ImageConstPtr& img_msg,const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::cout<<"In LidarImageCallbackFunction():Enter callback function."<<std::endl;
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"vision_lidar_fusion_node");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/gi/simulation/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/velodyne_points2", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, lidar_sub);
    sync.registerCallback(boost::bind(&LidarImageCallbackFunction, _1, _2));


    //PrevFusion("config.yaml");
    //PrevFusion.projectLidarToRGBImageForVisualization();
    ros::spin();
    return 0;
}



