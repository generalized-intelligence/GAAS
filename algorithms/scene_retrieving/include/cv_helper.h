//
// Computer Vision Helper Class aimed at easing relevant computer vision tasks
//
// Created by gishr on 19-3-29.
//

#ifndef SCENE_RETRIEVING_CV_HELPER_H
#define SCENE_RETRIEVING_CV_HELPER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>


#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
//#include <opencv2/features2d.hpp>
#include <iostream>




using namespace std;
using namespace Eigen;


class cv_helper{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // class initializer
    cv_helper(const float &_fx, const float &_fy, const float &_cx, const float &_cy, const float _bf = 0)
    : fx(_fx), fy(_fy), cx(_cx), cy(_cy), bf(_bf)
    {
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        fxinv = 1 / fx;
        fyinv = 1 / fy;
        Kinv = K.inverse();
        f = (fx + fy) * 0.5;
        b = bf / f;
    }


    // project image points to world frame given camera K and current frame R and t
    vector<cv::Point3f> image2world(vector<cv::Point2f> image_points, vector<float> points_disparity, cv::Mat R, cv::Mat t) {

        assert(image_points.size() == points_disparity.size());

        cv::Point3f camera_point;
        vector<cv::Point3f> map_points;

        for(size_t i=0; i<image_points.size(); i++)
        {

            cv::Point2f pt = image_points[i];
            camera_point.x = (pt.x - cx) * fxinv * points_disparity[i];
            camera_point.y = (pt.y - cy) * fyinv * points_disparity[i];
            camera_point.z = points_disparity[i];

            Matrix3f R_eigen;
            Vector3f t_eigen;
            Vector3f pt_eigen;
            Vector3f map;

            cv::cv2eigen(R, R_eigen);
            cv::cv2eigen(t, t_eigen);

            pt_eigen[0] = camera_point.x;
            pt_eigen[1] = camera_point.y;
            pt_eigen[2] = camera_point.z;

            map = (R_eigen * pt_eigen + t_eigen);
            camera_point.x = map[0];
            camera_point.y = map[1];
            camera_point.z = map[2];

            map_points.push_back(camera_point);
        }

        return map_points;
    }

    // project image points to camera frame
    vector<cv::Point3f> image2cam(vector<cv::Point2f> image_points, vector<float> points_disparity) {

        assert(image_points.size() == points_disparity.size());

        cv::Point3f camera_point;
        vector<cv::Point3f> cam_points;

        for(size_t i=0; i<image_points.size(); i++)
        {

            cv::Point2f pt = image_points[i];
            camera_point.x = (pt.x - cx) * fxinv * points_disparity[i];
            camera_point.y = (pt.y - cy) * fyinv * points_disparity[i];
            camera_point.z = points_disparity[i];

            cam_points.push_back(camera_point);
        }

        return cam_points;
    }


    pcl::PointCloud<pcl::PointXYZ> PtsVec2PointCloud(vector<cv::Point3f> input_vector)
    {
        assert(!input_vector.empty());

        pcl::PointCloud<pcl::PointXYZ> cloud;

        cloud.width = input_vector.size();
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        for(size_t i=0; i< input_vector.size(); i++)
        {
            cloud.points[i].x = input_vector[i].x;
            cloud.points[i].y = input_vector[i].y;
            cloud.points[i].z = input_vector[i].z;
        }

        return cloud;
    }

    // a wrapper for pcl::GeneralizedIterativeClosestPoint to return the transformation matrix between a input point cloud and a
    // target point cloud
    Eigen::Matrix4f GeneralICP(vector<cv::Point3f> input_cloud, vector<cv::Point3f> target_cloud, int num_iter = 50, double transformationEpsilon = 1e-8)
    {

        typedef pcl::PointXYZ PointT;

        // define input point cloud
        pcl::PointCloud<PointT>::Ptr src (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT> src_cloud = PtsVec2PointCloud(input_cloud);
        *src  = src_cloud;

        // define target point cloud
        pcl::PointCloud<PointT>::Ptr tgt (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT> tgt_cloud = PtsVec2PointCloud(input_cloud);
        *tgt = tgt_cloud;

        // define output point cloud
        pcl::PointCloud<PointT> output;

        // parameter setter
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
        reg.setInputSource(src);
        reg.setInputTarget(tgt);
        reg.setMaximumIterations(num_iter);
        reg.setTransformationEpsilon(transformationEpsilon);

        reg.align (output);

        // expect fitness score is less than a certain value
        cout<<"GeneralICP::General ICP fitness score: "<<reg.getFitnessScore()<<endl;

        Eigen::Matrix4f transformation;
        transformation = reg.getFinalTransformation();

        cout<<"GeneralICP::transformation matrix: "<<transformation<<endl;
    }



public:

    float fx = 0;
    float fy = 0;
    float cx = 0;
    float cy = 0;
    float fxinv = 0;
    float fyinv = 0;
    float b = 0;
    float f = 0;
    float bf = 0;

    Eigen::Matrix3f K = Matrix3f::Identity();     // intrinsics
    Eigen::Matrix3f Kinv = Matrix3f::Identity();  // inverse K

};






















#endif //SCENE_RETRIEVING_CV_HELPER_H
