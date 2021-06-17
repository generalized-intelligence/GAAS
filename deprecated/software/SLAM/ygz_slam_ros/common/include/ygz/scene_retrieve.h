#ifndef SCENE_RETRIEVE_H
#define SCENE_RETRIEVE_H

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
#include "ygz/scene_frame_properties.h"
#include "ygz/serialization.h"

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "ygz/cv_helper.h"


using namespace std;
using namespace cv;

                    //pts2d_in    pts3d_in    desp,   R,    t
typedef  std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::Point3d>, cv::Mat, cv::Mat, cv::Mat> SceneFrame;

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
    
    inline void addFrame(const std::vector<cv::KeyPoint>& points2d_in, const std::vector<cv::Point3d>points3d_in, const cv::Mat& point_desp_in, const cv::Mat R, const cv::Mat t)
    {
        cout<<"Adding frame: "<<R<<t<<endl;
        
        this->vec_p2d.push_back(points2d_in);
        this->vec_p3d.push_back(points3d_in);
        this->point_desps.push_back(point_desp_in);
        
        this->mVecR.push_back(R);
        this->mVecT.push_back(t);
        
        this->mIndex++;
    }
    
    
    inline void addFrame(const SceneFrame& frame)
    {
        this->addFrame(std::get<0>(frame), std::get<1>(frame), std::get<2>(frame), std::get<3>(frame), std::get<4>(frame));
    }
    
    
    inline void setHasScale(bool hasScale_in)
    {
        this->hasScale = hasScale_in;
    }
    

    void saveVoc();


    void saveFile(const std::string &filename);

    
    void loadFile(const std::string &filename);
    
    
    void test();

    SceneFrame generateSceneFrameFromStereoImage(cv::Mat imgl, cv::Mat imgr, cv::Mat RotationMat, cv::Mat TranslationMat, cv::Mat Q_mat);
    
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
        
        ar & mVecR;
        ar & mVecT;
        
        //ar & m_RT_Scene_Fix;
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
        
        ar & mVecR;
        ar & mVecT;
        
        //ar & m_RT_Scene_Fix;
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
    
    vector<cv::Mat> mVecR;
    vector<cv::Mat> mVecT;
    
    std::vector<SceneFrame_Properties> vec_properties;
    
    cv::Mat m_RT_Scene_Fix = cv::Mat::eye(4,4,CV_32F);//fix 3d pose of scene.

    cv_helper* mpCv_helper = nullptr;

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_of_scene; //Take care:this cloud is not required, so do not use it in any algorithm.
};

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


//SceneFrame generateSceneFrameFromStereoImage(cv::Mat imgl, cv::Mat imgr, cv::Mat RotationMat, cv::Mat TranslationMat, cv::Mat Q_mat)
//{
//
//    cout<<"generateSceneFrameFromStereoImage 1"<<endl;
//
//    cv_helper* mpCv_helper = new cv_helper(360.0652, 363.2195, 406.6650, 256.2053, 39.9554);
//
//    LoopClosingManager lcm("./voc/brief_k10L6.bin");
//
//    std::vector<cv::KeyPoint> key_points2d_candidate;
//    std::vector<cv::KeyPoint> key_points2d_final;
//    std::vector<cv::Point3d> points3d;
//    cv::Mat feature;
//
//    ptr_frameinfo pleft_image_info = lcm.extractFeature(imgl);
//    ptr_frameinfo pright_image_info = lcm.extractFeature(imgr);
//
//    cv::Mat feature_l,feature_r;
//    feature_l = pleft_image_info->descriptors;
//    feature_r = pright_image_info->descriptors;
//
//    std::vector<int> good_2dmatches_index;
//
//    FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12,20,2));
//    //FlannBasedMatcher matcher = FlannBasedMatcher();
//
//    std::vector< DMatch > matches;
//    matcher.match(feature_l, feature_r, matches);
//
//    double max_dist = 0;
//    double min_dist = 100;
//    for( int i = 0; i < matches.size(); i++ )
//    {
//        double dist = matches[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }
//
//    std::vector< DMatch > good_matches;
//    for( int i = 0; i < matches.size(); i++ )
//    {
//        if( matches[i].distance <= 2*min_dist && matches[i].distance< 10) // 3.0 too large;2.0 too large.
//        {
//            good_matches.push_back( matches[i]);
//        }
//    }
//
//    std::vector<Point2f> lk_input_keypoints, lk_output_keypoints;
//    for(size_t i = 0; i < good_matches.size(); i++)
//    {
//        lk_input_keypoints.push_back(pleft_image_info->keypoints[good_matches[i].queryIdx].pt);//will check if LKFlow exist.
//        good_2dmatches_index.push_back(good_matches[i].queryIdx);
//    }
//
//
//    std::vector<unsigned char> PyrLKResults;
//    std::vector<float> optflow_err;
//
//    //Size winSize(31,31);
//    //TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//
//    cout<<"lk_input_keypoints.size(): "<<(lk_input_keypoints.size())<<endl;
//
//    if (lk_input_keypoints.size()<=5)
//    {
//        SceneFrame failed;
//        return failed;
//    }
//
//    cv::calcOpticalFlowPyrLK(imgl,
//                            imgr,
//                            lk_input_keypoints,
//                            lk_output_keypoints,
//                            PyrLKResults,
//                            optflow_err);
//
//
//    cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size())<<endl;
//    cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size())<<endl;
//
//
//    if (lk_input_keypoints.size() < 5 || lk_output_keypoints.size() < 5 || PyrLKResults.size() < 5)
//    {
//        cout<<"lk_input_keypoints.size() == 0: "<<(lk_input_keypoints.size() == 0)<<endl;
//        cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size() == 0)<<endl;
//        cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size() == 0)<<endl;
//    }
//
//
//    std::vector<Point2f> matched_points;
//    std::vector<float> disparity_of_points;
//    cv::Mat descriptors_reserved;
//
//
//    for(int index = 0; index<lk_input_keypoints.size(); index++)
//    {
//        if(PyrLKResults[index] == 1)
//        {
//            matched_points.push_back(lk_input_keypoints[index]);
//            disparity_of_points.push_back(lk_input_keypoints[index].x - lk_output_keypoints[index].x);
//
//            //Mat desp_reserved = pleft_image_info->descriptors.colRange(good_2dmatches_index[index], good_2dmatches_index[index]+1).clone().reshape(0);
//            Mat desp_reserved = pleft_image_info->descriptors.row(good_2dmatches_index[index]).clone().reshape(0);
//            descriptors_reserved.push_back(desp_reserved);
//        }
//    }
//
//    cout<<"Q_mat: "<<Q_mat<<endl;
//
//    std::vector<cv::Point3f> points3f;
//
////    points3f = mpCv_helper->image2world(lk_input_keypoints, disparity_of_points, RotationMat, TranslationMat);
////
////    points3d = mpCv_helper->Points3f2Points3d(points3f);
//
//
//    delete mpCv_helper;
//
//    cv::reprojectImageTo3D(disparity_of_points, points3f, Q_mat);
//
//
//    //do rotation and translation to points3d.
//    for(int i = 0;i<points3d.size();i++)
//    {
//        cv::Mat point3d_temp(points3d[i]);
//
//        Eigen::Matrix3f rotation;
//        Eigen::Vector3f translation;
//        Eigen::Vector3f point;
//        Eigen::Vector3f result;
//
//        cv::cv2eigen(RotationMat, rotation);
//        cv::cv2eigen(TranslationMat, translation);
//        cv::cv2eigen(point3d_temp, point);
//
//        result = rotation * point + translation;
//
//        //cv::Mat transformed = RotationMat*point3d_temp + TranslationMat;
//
//        cv::Mat transformed;
//        cv::eigen2cv(result, transformed);
//
//        Point3d output;
//        output.x = transformed.at<float>(0);
//        output.y = transformed.at<float>(1);
//        output.z = transformed.at<float>(2);
//        points3d[i] = output;//do transform in mat form.
//    }
//
//    //do rotation and translation to points3d.
////    for(int i = 0; i<points3f.size();i++)
////    {
////        cv::Mat point3d_temp(points3f[i]);
////
////        Eigen::Matrix3f rotation;
////        Eigen::Vector3f translation;
////        Eigen::Vector3f point;
////        Eigen::Vector3f result;
////
////        cv::cv2eigen(RotationMat, rotation);
////        cv::cv2eigen(TranslationMat, translation);
////        cv::cv2eigen(point3d_temp, point);
////
////        result = rotation * point + translation;
////
////        //cv::Mat transformed = RotationMat*point3d_temp + TranslationMat;
////
////        cv::Mat transformed;
////        cv::eigen2cv(result, transformed);
////
////        Point3d output;
////        output.x = transformed.at<float>(0);
////        output.y = transformed.at<float>(1);
////        output.z = transformed.at<float>(2);
////        points3d.push_back(output);//do transform in mat form.
////    }
//
//
//    cv::KeyPoint::convert(matched_points, key_points2d_final);
//
//    cout<<"generate scene, points3d size: "<<points3d.size()<<endl;
//
//
//    return std::make_tuple(key_points2d_final, points3d, descriptors_reserved, RotationMat, TranslationMat);
//
//}

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
#endif
