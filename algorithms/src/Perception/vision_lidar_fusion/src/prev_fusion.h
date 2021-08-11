#ifndef FUSION_POINTCLOUD_H
#define FUSION_POINTCLOUD_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using std::string;
using std::vector;
using std::cout;
using std::endl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> LidarCloudT;

class PrevFusion
{
    cv::Mat camera_K;
    cv::Mat T_cam_lidar;
public:
    PrevFusion(const string& file_path)
    {
        cv::FileStorage fs;
        fs.open(file_path,cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            LOG(ERROR)<<"File "<< file_path<<" not found!"<<endl;
            exit(-1);
        }
        fs["CameraK"]>>camera_K;
        fs["T_cam_lidar"]>>T_cam_lidar;
        assert(camera_K.cols == 3 && camera_K.rows==3 && camera_K.type()==CV_32F);
        assert(T_cam_lidar.cols == 4 && T_cam_lidar.rows==4 && T_cam_lidar.type()==CV_32F);
    }
    void reprojectColoredImageTo3d();
    void projectLidarToRGBImageForVisualization(const cv::Mat& image_rect,const LidarCloudT& lidar_cloud,vector<cv::Point2f>& vPt2ds_output,bool do_visualize=false)//,cv::Mat& output_img)
    {
        cv::Mat img;
        if(image_rect.channels() == 1)
        {
            cv::cvtColor(image_rect,img,cv::COLOR_BGR2GRAY);
        }
        else
        {
            img = image_rect.clone();
        }
        cv::Mat cvPoints3dMat(4,lidar_cloud.size(),CV_32F,0.0); //T_cam_lidar*P_lidar = P_cam.

        for(int i = 0;i<lidar_cloud.size();i++)
        {
            const PointT &pt = lidar_cloud.points.at(i);
            cvPoints3dMat.at<float>(0,i) = pt.x;
            cvPoints3dMat.at<float>(1,i) = pt.y;
            cvPoints3dMat.at<float>(2,i) = pt.z;
            cvPoints3dMat.at<float>(3,i) = 1.0;
        }
        cv::Mat PCamMat = T_cam_lidar*cvPoints3dMat;
        vector<cv::Point3f> vPt3ds;
        vPt3ds.reserve(lidar_cloud.size());
        for(int i = 0;i<lidar_cloud.size();i++)
        {
            cv::Point3f pt;
            pt.x = PCamMat.at<float>(0,i);
            pt.y = PCamMat.at<float>(1,i);
            pt.z = PCamMat.at<float>(2,i);
            vPt3ds.push_back(pt);
        }
        vector<cv::Point3f> vPt3ds_z_gt_0;//check z>=0 in camera coordinate.

        for(const auto& pt:vPt3ds)
        {
            if(pt.z>0)
            {
                vPt3ds_z_gt_0.push_back(pt);
            }
        }
        LOG(INFO)<<"vPt3ds_z_gt_0.size():"<<vPt3ds_z_gt_0.size()<<endl;
        vector<cv::Point2f>& vPt2ds_projected = vPt2ds_output;
        vPt2ds_projected.clear();
        cv::Mat rvec=(cv::Mat_<double>(3,1) <<0,0,0);
        cv::Mat tvec=(cv::Mat_<double>(3,1) <<0,0,0);
        cv::Mat distortvec=(cv::Mat_<double>(4,1) <<0,0,0,0);
        if(vPt3ds_z_gt_0.size()>0)
        {
            cv::projectPoints(vPt3ds_z_gt_0,rvec,tvec,camera_K,distortvec,vPt2ds_projected);
        }
        vPt2ds_output = vPt2ds_projected;
        LOG(INFO)<<"p2d.size():"<<vPt2ds_output.size()<<endl;
        if(do_visualize)
        {
            for(const cv::Point2f& pt2d:vPt2ds_projected)
            {
                int x = (int)pt2d.x;
                int y = (int)pt2d.y;
                if(x<0||x>=img.cols||y<0||y>img.rows)
                {
                    continue;
                }
                cv::Point2i center;
                center.x = x;
                center.y = y;
                //cout<<"x,y:"<<x<<" "<<y<<endl;
                cv::circle(img,center,3,127,-1);
            }
            cv::imshow("Projected Image",img);
            cv::waitKey(1);
        }
    }

};




#endif
