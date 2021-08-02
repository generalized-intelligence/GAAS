#ifndef GROUND_POINT_REMOVAL_TYPEDEFS_H
#define GROUND_POINT_REMOVAL_TYPEDEFS_H

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <deque>
#include <Eigen/Core>

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
