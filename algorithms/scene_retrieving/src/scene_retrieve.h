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

typedef  std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::Point3d>,cv::Mat> SceneFrame;

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
    inline void addFrame(const std::vector<cv::KeyPoint>& points2d_in,const std::vector<cv::Point3d>points3d_in,const cv::Mat& point_desp_in)
    {
        this->vec_p2d.push_back(points2d_in);
        this->vec_p3d.push_back(points3d_in);
        this->point_desps.push_back(point_desp_in);
    }
    inline void addFrame(const SceneFrame& frame)
    {
        this->addFrame(std::get<0>(frame),std::get<1>(frame),std::get<2>(frame))
    }
    inline void setHasScale(bool hasScale_in)
    {
        this->hasScale = hasScale_in;
    }
private:
  
    bool hasScale = false;
    std::vector<std::vector<cv::KeyPoint>> vec_p2d;
    std::vector<std::vector <cv::Point3d>> vec_p3d;
    std::vector <cv::Mat> point_desps;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_of_scene;//Take care:this cloud is not required, so do not use it in any algorithm.
    cv::Mat m_RT_Scene_Fix = cv::Mat::eye(4,4,CV_32F);//fix 3d pose of scene.
};

std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::Point3d>,cv::Mat> generateSceneFrameFromStereoImage(const cv::Mat &imgl,cv::Mat &imgr)
{
    std::vector<cv::KeyPoint> key_points2d_candidate;
    std::vector<cv::KeyPoint> key_points2d_final;
    std::vector<cv::Point3d> points3d;
    cv::Mat feature;
    ptr_frameinfo pleft_image_info = LoopClosingManager::extractFeature(imgl);
    ptr_frameinfo pright_image_info = LoopClosingManager::extractFeature()
    
    cv::Mat feature_l,feature_r;
    feature_l = pleft_image_info->descriptors;
    feature_r = pright_image_info->descriptors;
    
    
    std::vector<int> good_2dmatches_index;
    
    FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12,20,2));
    //FlannBasedMatcher matcher = FlannBasedMatcher();
    std::vector< DMatch > matches;
    //matcher.match( kdesp_list[index1], kdesp_list[index2], matches );
    matcher.match(feature_l, feature_r, matches);
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    std::vector< DMatch > good_matches;
    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= 2*min_dist && matches[i].distance< ORB_TH_HIGH) // 3.0 too large;2.0 too large.
        {
            good_matches.push_back( matches[i]); 
        }
    }
    
    std::vector<Point2f> lk_input_keypoints,lk_output_keypoints;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {   
        lk_input_keypoints.push_back(pleft_image_info->keypoints[good_matches[i].queryIdx].pt);//will check if LKFlow exist.
	good_2dmatches_index.push_back(good_matches[i].queryIdx);
    }
    
    
    std::vector<unsigned char> PyrLKResults;
    cv::calcOpticalFlowPyrLK( imgl,
		imgr,
		lk_input_keypoints,
		lk_output_keypoints,
		PyrLKResults,
		//OutputArray  	err,
		//Size  	winSize = Size(21, 21),
		//int  	maxLevel = 3,
		//TermCriteria  	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
		//int  	flags = 0,
		//double  	minEigThreshold = 1e-4 
	);
    std::vector<Point2f> matched_points;
    std::vector<float> disparity_of_points;
    cv::Mat descriptors_reserved;
    for(int index = 0;index<lk_input_keypoints.size();index++)
    {
        if(PyrLKResults[index] == 1)
	{
	  matched_points.push_back(lk_input_keypoints[index]);
	  disparity_of_points.push_back(lk_output_keypoints[index][0]-lk_input_keypoints[index][0]);
	  descriptors_reserved.push_back(pleft_image_info->descriptors[good_2dmatches_index[index]]);
	  //push_back(pleft_image_info->descriptors[good_2dmatches_index[index]]);
	}
    }
    cv::reprojectImageTo3D(disparity_of_points,points3d,Q_mat);
    cv::KeyPoint::convert(matched_points,key_points2d_final);
    return std::make_tuple<>(key_points2d_final,points3d,descriptors_reserved);
}

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
