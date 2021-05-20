#ifndef MULTISTAGE_TYPEDEFS_H
#define MULTISTAGE_TYPEDEFS_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <deque>

using std::string;
using std::vector;
using std::deque;
using std::cout;
using std::endl;
typedef pcl::PointXYZI LidarPointT;
typedef pcl::PointCloud<LidarPointT> LidarCloudT;
typedef pcl::PointXYZI MapPointT;
typedef pcl::PointCloud<MapPointT> MapCloudT;




#endif
