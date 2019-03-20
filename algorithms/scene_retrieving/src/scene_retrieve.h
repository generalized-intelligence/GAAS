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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

//#include <pcl_conversions/pcl_conversions.h"
#include<pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "LoopClosingManager.h"


#include "serialization.h"


using namespace std;
using namespace cv;

typedef  std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::Point3d>,cv::Mat> SceneFrame;

class Scene  // a scene is divided into multi images with depth.the point3d represents its position in the axis of the whole scene ,not the 3d position in its image.
{
public:
    Scene();

    void fromCVMat(std::vector<std::vector<cv::KeyPoint>> points_2d,const std::vector<std::vector<cv::Point3d>>& points_3d,const std::vector<Mat>& point_desps);
    
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
        
        this->vec_p2d.push_back(vector<cv::KeyPoint>(points2d_in));
        this->vec_p3d.push_back(vector<cv::Point3d>(points3d_in));
        this->point_desps.push_back(cv::Mat(point_desp_in));
        this->mIndex++;
    }
    
    inline void addFrame(const SceneFrame& frame)
    {
        this->addFrame(std::get<0>(frame),std::get<1>(frame),std::get<2>(frame));
    }
    
    inline void setHasScale(bool hasScale_in)
    {
        this->hasScale = hasScale_in;
    }
    


    void saveFile(const std::string &filename)
    {
        
        std::ofstream ofs(filename);

        {
            boost::archive::text_oarchive oa(ofs);
            oa << *this;
        }
    }


    Scene loadFile(const std::string &filename)
    {

        std::ifstream ifs(filename);

        {
            boost::archive::text_iarchive ia(ifs);
            ia >> *this;
            cout << "Deserialization finished" << endl;
        }

        return RecovedredScene;
    }
    
    
    /////////////////////////////////// serialization////////////////////////////////////
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    template <class Archive>
    void save (Archive & ar, const unsigned int version) const
    {
        cout << "scene saving started!" << endl;

        ar & mIndex;
        ar & hasScale;
        ar & vec_p2d;
        ar & vec_p3d;
        ar & point_desps;
        ar & m_RT_Scene_Fix;

        //ar & point_cloud_of_scene;
        cout << "scene saving finished!" << endl;
    }

    template <class Archive>
    void load (Archive & ar, const unsigned int version)
    {
        cout << "scene loading started!" << endl;

        ar & mIndex;
        ar & hasScale;
        ar & vec_p2d;
        ar & vec_p3d;
        ar & point_desps;
        ar & m_RT_Scene_Fix;

        //ar & point_cloud_of_scene;

        cout << "scene loading finished!" << endl;
    }
    /////////////////////////////////// serialization////////////////////////////////////


    bool hasScale = false;


private:

    size_t mIndex = 0;
    std::vector<std::vector<cv::KeyPoint>> vec_p2d;
    std::vector<std::vector <cv::Point3d>> vec_p3d;
    std::vector <cv::Mat> point_desps;
    cv::Mat m_RT_Scene_Fix = cv::Mat::eye(4,4,CV_32F);//fix 3d pose of scene.

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_of_scene; //Take care:this cloud is not required, so do not use it in any algorithm.
};


SceneFrame generateSceneFrameFromStereoImage(const cv::Mat &imgl,cv::Mat &imgr,const cv::Mat& RotationMat,const cv::Mat& TranslationMat,const cv::Mat& Q_mat)
{
    std::vector<cv::KeyPoint> key_points2d_candidate;
    std::vector<cv::KeyPoint> key_points2d_final;
    std::vector<cv::Point3d> points3d;
    cv::Mat feature;
    ptr_frameinfo pleft_image_info = LoopClosingManager::extractFeature(imgl);
    ptr_frameinfo pright_image_info = LoopClosingManager::extractFeature(imgr);
    
    cv::Mat feature_l,feature_r;
    feature_l = pleft_image_info->descriptors;
    feature_r = pright_image_info->descriptors;
    
    
    std::vector<int> good_2dmatches_index;
    
    FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12,20,2));
    //FlannBasedMatcher matcher = FlannBasedMatcher();
    std::vector< DMatch > matches;
    //matcher.match( kdesp_list[index1], kdesp_list[index2], matches );
    matcher.match(feature_l, feature_r, matches);
    double max_dist = 0; 
    double min_dist = 100;
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
    }//LoopClosingManager
    
    
    std::vector<unsigned char> PyrLKResults;
    std::vector<float> optflow_err;
    //cv::Size_<int> optflow_err;
    
    //Size winSize(31,31);
    //TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    
    cv::calcOpticalFlowPyrLK(imgl,
		imgr,
		lk_input_keypoints,
		lk_output_keypoints,
		PyrLKResults,
		optflow_err);//,
		//winSize,3, termcrit, 0, 0.001);
    
    std::vector<Point2f> matched_points;
    std::vector<float> disparity_of_points;
    cv::Mat descriptors_reserved;
    for(int index = 0;index<lk_input_keypoints.size();index++)
    {
        if(PyrLKResults[index] == 1)
	{
	  matched_points.push_back(lk_input_keypoints[index]);
	  disparity_of_points.push_back(lk_output_keypoints[index].x-lk_input_keypoints[index].x);
	  Mat desp_reserved = pleft_image_info->descriptors.colRange(good_2dmatches_index[index],good_2dmatches_index[index]+1).clone().reshape(0);
	  descriptors_reserved.push_back(desp_reserved);//get this row out of mat.
	  //push_back(pleft_image_info->descriptors[good_2dmatches_index[index]]);
	}
    }
    cv::reprojectImageTo3D(disparity_of_points,points3d,Q_mat);
    //do rotation and translation to points3d.
    for(int i = 0;i<points3d.size();i++)
    {
        cv::Mat point3d_temp(points3d[i]);
	
	cv::Mat transformed = RotationMat*point3d_temp + TranslationMat;
	Point3d output;
	output.x = transformed.at<double>(0);
	output.y = transformed.at<double>(1);
	output.z = transformed.at<double>(2);
        points3d[i] = output;//do transform in mat form.
    }
    cv::KeyPoint::convert(matched_points,key_points2d_final);
    return std::make_tuple<>(key_points2d_final,points3d,descriptors_reserved);
}

class SceneRetriever
{
public:

    SceneRetriever();

    SceneRetriever(const string& voc);
    //SceneRetriever(Scene& original_scene_input);
    SceneRetriever(const string&voc,const std::string& scene_file);
    int retrieveSceneWithScaleFromMonoImage(const cv::Mat image_in_rect,const cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success);
    
    int retrieveSceneWithMultiStereoCam(const std::vector<cv::Mat> leftCams,const std::vector<cv::Mat> rightCams,
				      std::vector<cv::Mat> RT_pose_of_stereo_cams,
				      cv::Mat &RT_mat_of_multi_stereo_cam_output,
				      bool &match_success
				       );
    int retrieveSceneWithMultiMonoCam(const std::vector<cv::Mat> images,std::vector<cv::Mat> RT_pose_of_mono_cams,cv::Mat &RT_mat_of_multi_mono_cam_output,bool& match_success);
    int retrieveSceneFromStereoImage(const cv::Mat image_left_rect, const cv::Mat image_right_rect, const cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success);
    
    
    
    std::pair<std::vector<std::vector<DMatch>>,std::vector<int>> matchImageWithScene2D(const cv::Mat image);
    void debugVisualize();//visualize pointcloud and cam pose.
    
    
private:
    void _init_retriever();
    Scene original_scene;
    LoopClosingManager* ploop_closing_manager_of_scene;
};
