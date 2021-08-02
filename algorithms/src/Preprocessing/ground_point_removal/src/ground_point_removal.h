#ifndef GROUND_POINT_REMOVAL_H
#define GROUND_POINT_REMOVAL_H


#include "typedefs.h"

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <glog/logging.h>


void removeGround(LidarCloudT::Ptr input_cloud,LidarCloudT::Ptr& output_cloud,double maxDist=0.3)
{
    if(maxDist<0)
    {
        LOG(ERROR)<<"[ground_point_removal]Illegal Maxdist."<<endl;
        return;
    }
    output_cloud = LidarCloudT::Ptr(new LidarCloudT);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices);

    pcl::SACSegmentation<LidarPointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    //seg.setMaxIterations();

    // by default:
    //max_iterations_ (50)
    //probability_ (0.99)

    seg.setDistanceThreshold (maxDist);
    seg.setInputCloud (input_cloud);
    seg.segment (*inliers_indices, *coefficients);
    if (inliers_indices->indices.size() == 0)
    {
        LOG(ERROR)<<"[ground_point_removal] No plane estimation."<<endl;
        output_cloud = input_cloud;
        return;
    }
    pcl::ExtractIndices<LidarPointT> ext;
    ext.setInputCloud (input_cloud);
    ext.setIndices(inliers_indices);
    ext.setNegative (true);
    ext.filter (*output_cloud);
}

#endif
