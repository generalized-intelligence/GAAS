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
    vector<cv::Point3f> image2world(vector<cv::Point2f>& image_points, vector<float>& points_disparity, cv::Mat& R, cv::Mat& t) {

        assert(image_points.size() == points_disparity.size());

        cout<<"image2world 1"<<endl;

        cv::Point3f camera_point, map_point;
        vector<cv::Point3f> map_points;

        cout<<"image2world 2"<<endl;

        for(size_t i=0; i<image_points.size(); i++)
        {
            cv::Point2f pt = image_points[i];

            //convert disparity to depth
            points_disparity[i] = (this->bf) / (points_disparity[i] + 1e-5);

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
            map_point.x = map[0];
            map_point.y = map[1];
            map_point.z = map[2];

            map_points.push_back(map_point);
        }

        cout<<"image2world 3"<<endl;

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

            points_disparity[i] = (this->bf) / (points_disparity[i] + 1e-5);

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


    vector<cv::Point3f> Points3d2Points3f(vector<cv::Point3d> input_points)
    {
        vector<cv::Point3f> output_points;

        cv::Point3f temp_p;
        for(auto& p : input_points)
        {
            temp_p.x = p.x;
            temp_p.y = p.y;
            temp_p.z = p.z;

            output_points.push_back(temp_p);
        }

        return output_points;
    }

    vector<cv::Point3d> Points3f2Points3d(vector<cv::Point3f> input_points)
    {
        vector<cv::Point3d> output_points;

        cv::Point3d temp_p;
        for(auto& p : input_points)
        {
            temp_p.x = p.x;
            temp_p.y = p.y;
            temp_p.z = p.z;

            output_points.push_back(temp_p);
        }

        return output_points;
    }


    // print type of cv::Mat

    string type2str(int type) {
        string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
            case CV_8U:  r = "8U"; break;
            case CV_8S:  r = "8S"; break;
            case CV_16U: r = "16U"; break;
            case CV_16S: r = "16S"; break;
            case CV_32S: r = "32S"; break;
            case CV_32F: r = "32F"; break;
            case CV_64F: r = "64F"; break;
            default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
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
