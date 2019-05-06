//
// Created by gishr on 19-4-12.
//

#ifndef STEREO_DENSE_RECONSTRUCTION_PCL_HELPER_H
#define STEREO_DENSE_RECONSTRUCTION_PCL_HELPER_H


#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/PointField.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Point32.h>

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"


using namespace std;
using namespace cv;
using namespace cv::ximgproc;

class pcl_helper
{

public:

    pcl_helper();


    //PCL related pointcloud helper functions
    void ROSPointCloud2toPointCloudXYZ(sensor_msgs::PointCloud2& input_cloud, pcl::PointCloud<pcl::PointXYZ>& output_cloud);

    void PointCloudXYZtoROSPointCloud2(pcl::PointCloud<pcl::PointXYZ>& input_cloud, sensor_msgs::PointCloud2& output_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr PCLStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_p, int mean_k=50, int stdThres=1);

    sensor_msgs::PointField sensorPointField(string name_of_field, int offset, int datatype, int elementNum);

//    sensor_msgs::PointCloud2* createPointCloud2(vector<geometry_msgs::Point32>& input_points);

    bool createPointCloud2(vector<geometry_msgs::Point32>& input_points, sensor_msgs::PointCloud2& pc);


    //OPENCV related pointcloud helper functions





};



#endif //STEREO_DENSE_RECONSTRUCTION_PCL_HELPER_H
