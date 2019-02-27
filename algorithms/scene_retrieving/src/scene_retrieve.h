#include <iostream>
#include <vector>

// DBoW2/3
#include "DBoW3.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "LoopClosingManager.h"

class Scene  // a scene is divided into multi images with depth.the point3d represents its position in the axis of the whole scene ,not the 3d position in its image.
{
public:
    Scene();
    void fromCVMat(std::vector<std::vector<cv::KeyPoint>> points_2d,const std::vector<std::vector<cv::Point3d>>& points_3d,const std::vector<Mat>& point_desps);
    void saveFile(const std::string &filename)
    {
      ;
    }
    void loadFile(const std::string &filename)
    {
      ;
    }
    void RotateAndTranslate(const Mat &RT)
    {
      ;
    }
    void setVisiblePointCloud(const std::string &pointcloud_filename);
    inline int getImageCount();
    inline cv::Mat& getDespByIndex(int i)
    {
      return this->point_desps[i];
    }
    inline std::vector<std::vector<cv::KeyPoint>>& getP2D()
    {
      return this->vec_p2d;
    }
    inline std::vector<std::vector<cv::Point3d>>& getP3D()
    {
      return this->vec_p3d;
    }
private:
  
    bool hasScale = false;
    std::vector<std::vector<cv::KeyPoint>> vec_p2d;
    std::vector<std::vector <cv::Point3d>> vec_p3d;
    std::vector <cv::Mat> point_desps;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_of_scene;//Take care:this cloud is not required, so do not use it in any algorithm.
    cv::Mat m_RT_Scene_Fix = cv::Mat::eye(4,4,CV_32F);//fix 3d pose of scene.
};

class SceneRetriever
{
public:
    SceneRetriever();
    SceneRetriever(Scene& original_scene_input);
    SceneRetriever(const std::string& scene_file);
    int retrieveSceneFromStereoImage(const cv::Mat image_left_rect,const cv::Mat image_right_rect,double camera_bf,
				      cv::Mat &RT_mat_of_stereo_cam_output,bool &match_success);
    int retrieveSceneWithScaleFromMonoImage(const cv::Mat image_in,cv::Mat&RT_mat_of_mono_cam_output,bool& match_success);
    
    int retrieveSceneWithMultiStereoCam(const std::vector<cv::Mat> leftCams,const std::vector<cv::Mat> rightCams,
				      std::vector<cv::Mat> RT_pose_of_stereo_cams,
				      cv::Mat &RT_mat_of_multi_stereo_cam_output,
				      bool &match_success
				       );
    int retrieveSceneWithMultiMonoCam(const std::vector<cv::Mat> images,std::vector<cv::Mat> RT_pose_of_mono_cams,cv::Mat &RT_mat_of_multi_mono_cam_output,bool& match_success);
    
    
    
    std::pair<std::vector<std::vector<DMatch>>,std::vector<int>> matchImageWithScene2D(const cv::Mat image);
    void debugVisualize();//visualize pointcloud and cam pose.
    
    
private:
    void _init_retriever();
    Scene original_scene;
    LoopClosingManager loop_closing_manager_of_scene;
};
