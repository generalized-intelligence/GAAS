#ifndef SCENE_RETRIEVE_H
#define SCENE_RETRIEVE_H

#include <iostream>
#include <vector>

// DBoW2/3
#include <glog/logging.h>
// DBoW2/3

#include "DBoW3.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d.hpp"


#include "LoopClosingManager.h"
#include "scene_frame_properties.h"
#include "serialization.h"

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>loopclosure_result
//#include <pcl/common/transforms.h>
//#include <pcl/registration/gicp.h>
//#include <pcl/conversions.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/io/pcd_io.h>
#include "cv_helper.h"


using namespace std;

                    //pts2d_in    pts3d_in    desp,   R,    t
typedef  std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::Point3d>, cv::Mat, cv::Mat, cv::Mat> SceneFrame;

class Scene  // a scene is divided into multi images with depth.the point3d represents its position in the axis of the whole scene ,not the 3d position in its image.
{
    
public:
    Scene();

    void fromCVMat(std::vector<std::vector<cv::KeyPoint>> points_2d,const std::vector<std::vector<cv::Point3d>>& points_3d,const std::vector<cv::Mat>& point_desps);
    
    void RotateAndTranslate(const cv::Mat &RT)
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
    inline int getCurrentIndex()
    {
        return this->mIndex;
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
    
    void saveDeserializedPoseToCSV();
    
    void test(bool savePosition=false);

    std::vector <cv::Point3d> fetchFrameMapPoints(size_t frame_index);

    void removeEmptyElement();



    /////////////////////////////////// serialization////////////////////////////////////
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    template <class Archive>
    void save (Archive & ar, const unsigned int version) const
    {
        cout << "scene saving started!" << endl;

        //size_t mIndex = 0;
        //std::vector<std::vector<cv::KeyPoint> > vec_p2d;
        //std::vector<std::vector<cv::Point3d> > vec_p3d;
        //std::vector <cv::Mat> point_desps;
        //vector<cv::Mat> mVecR;
        //vector<cv::Mat> mVecT;

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

    inline cv::Mat getR(size_t index)
    {
        return this->mVecR[index];
    }

    inline cv::Mat getT(size_t index)
    {
        return this->mVecT[index];
    }

    bool hasScale = false;

public:

    size_t mIndex = 0;
    std::vector<std::vector<cv::KeyPoint> > vec_p2d;
    std::vector<std::vector<cv::Point3d> > vec_p3d;
    std::vector <cv::Mat> point_desps;
    
    vector<cv::Mat> mVecR;
    vector<cv::Mat> mVecT;
    
    std::vector<SceneFrame_Properties> vec_properties;
    
    cv::Mat m_RT_Scene_Fix = cv::Mat::eye(4,4,CV_32F);//fix 3d pose of scene.

};




SceneFrame generateSceneFrameFromStereoImage(const cv::Mat &imgl, cv::Mat &imgr, const cv::Mat& RotationMat, const cv::Mat& TranslationMat, const cv::Mat& Q_mat)
{
    
    std::vector<cv::KeyPoint> key_points2d_candidate;
    std::vector<cv::KeyPoint> key_points2d_final;
    std::vector<cv::Point3d> points3d;
    cv::Mat feature;
    
    LoopClosingManager lcm("./voc/brief_k10L6.bin");

    ptr_frameinfo pleft_image_info = lcm.extractFeature(imgl);
    ptr_frameinfo pright_image_info = lcm.extractFeature(imgr);
    
    cv::Mat feature_l,feature_r;
    feature_l = pleft_image_info->descriptors;
    feature_r = pright_image_info->descriptors;
    
    std::vector<int> good_2dmatches_index;

    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12,20,2));
    //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<flann::IndexParams>(12,20,2));
    //cv::FlannBasedMatcher matcher = FlannBasedMatcher();

    std::vector< cv::DMatch > matches;
    matcher.match(feature_l, feature_r, matches);

    double max_dist = 0; 
    double min_dist = 100;
    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    std::vector< cv::DMatch > good_matches;
    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= 2*min_dist && matches[i].distance< 10) // 3.0 too large;2.0 too large.
        {
            good_matches.push_back( matches[i]); 
        }
    }
    
    
    std::vector<cv::Point2f> lk_input_keypoints, lk_output_keypoints;
    for(size_t i = 0; i < good_matches.size(); i++)
    {
        lk_input_keypoints.push_back(pleft_image_info->keypoints[good_matches[i].queryIdx].pt);//will check if LKFlow exist.
        good_2dmatches_index.push_back(good_matches[i].queryIdx);
    }


    if (lk_input_keypoints.size()<=5)
    {
        SceneFrame failed;
        return failed;
    }


    std::vector<unsigned char> PyrLKResults;
    std::vector<float> optflow_err;
    cv::calcOpticalFlowPyrLK(imgl,
                            imgr,
                            lk_input_keypoints,
                            lk_output_keypoints,
                            PyrLKResults,
                            optflow_err);
    
    cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size())<<endl;
    cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size())<<endl;
    
    if (lk_input_keypoints.size() < 5 || lk_output_keypoints.size() < 5 || PyrLKResults.size() < 5)
    {
        cout<<"lk_input_keypoints.size() == 0: "<<(lk_input_keypoints.size() == 0)<<endl;
        cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size() == 0)<<endl;
        cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size() == 0)<<endl;
        
//         SceneFrame temp;
//         return temp;
    }
        
    
    std::vector<cv::Point2f> matched_points;
    std::vector<float> disparity_of_points;
    cv::Mat descriptors_reserved;

    
    for(int index = 0; index<lk_input_keypoints.size(); index++)
    {
        if(PyrLKResults[index] == 1)
        {
            matched_points.push_back(lk_input_keypoints[index]);
            disparity_of_points.push_back(lk_output_keypoints[index].x-lk_input_keypoints[index].x);
            
            //Mat desp_reserved = pleft_image_info->descriptors.colRange(good_2dmatches_index[index], good_2dmatches_index[index]+1).clone().reshape(0);
            //Mat desp_reserved = pleft_image_info->descriptors.col(good_2dmatches_index[index]).clone().reshape(0);
            //descriptors_reserved.push_back(desp_reserved);//get this row out of mat.
            
            
            //push_back(pleft_image_info->descriptors[good_2dmatches_index[index]]);
        }
    }
    
    
    std::vector<cv::Point3f> points3f;
    
    cv::reprojectImageTo3D(disparity_of_points, points3f, Q_mat);
    
    
    
    //do rotation and translation to points3d.
    for(int i = 0;i<points3d.size();i++)
    {
        cv::Mat point3d_temp(points3d[i]);
        
        Eigen::Matrix3f rotation;
        Eigen::Vector3f translation;
        Eigen::Vector3f point;
        Eigen::Vector3f result;
        
        cv::cv2eigen(RotationMat, rotation);
        cv::cv2eigen(TranslationMat, translation);
        cv::cv2eigen(point3d_temp, point);
        
        result = rotation * point + translation;
        //cv::Mat transformed = RotationMat*point3d_temp + TranslationMat;
        
        cv::Mat transformed;
        cv::eigen2cv(result, transformed);

        cv::Point3d output;
        output.x = transformed.at<float>(0);
        output.y = transformed.at<float>(1);
        output.z = transformed.at<float>(2);
        points3d[i] = output;//do transform in mat form.
    }
    
    cv::KeyPoint::convert(matched_points,key_points2d_final);
    
    cout<<"key_points2d_final size: "<<key_points2d_final.size()<<endl;
    cout<<"points3d size: "<<points3d.size()<<endl;
    
    return std::make_tuple(key_points2d_final, points3d, descriptors_reserved, RotationMat, TranslationMat);
}



class SceneRetriever
{
public:
    
    SceneRetriever();
    
    SceneRetriever(const string& voc);
    
    //SceneRetriever(Scene& original_scene_input);
    SceneRetriever(const string&voc,const std::string& scene_file);
    
    float retrieveSceneWithScaleFromMonoImage(cv::Mat image_in_rect, cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,int* pMatchedIndexID_output = nullptr);
    float retrieveSceneFromStereoImage(cv::Mat& image_left_rect, cv::Mat& image_right_rect,
                                                       cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success,int* pMatchedIndexID_output = nullptr);
    inline int addFrameToScene(const std::vector<cv::KeyPoint>& points2d_in, const std::vector<cv::Point3d>points3d_in,const cv::Mat& point_desp_in, const cv::Mat R, const cv::Mat t)
    {
        if(!this->ploop_closing_manager_of_scene )
        {
            LOG(ERROR)<<"Calling addFrameToScene() without initiated loop closing manager!"<<endl;
            return -1;
        }
        this->original_scene.addFrame(points2d_in,points3d_in,point_desp_in,R,t);
        struct FrameInfo* pfr = new struct FrameInfo;
        pfr->keypoints = this->original_scene.getP2D()[this->original_scene.getCurrentIndex()];
        pfr->descriptors = this->original_scene.getDespByIndex(original_scene.getCurrentIndex());
        cout<<"pfr->keypoints size: "<<pfr->keypoints.size()<<endl;
        cout<<"pfr->descriptors size: "<<pfr->descriptors.size()<<endl;
        ptr_frameinfo frame_info(pfr);
        this->ploop_closing_manager_of_scene->addKeyFrame(frame_info);
        LOG(INFO)<<"addFrameToScene() successfully!";
        return 0;
    }

    
    
    float retrieveSceneWithMultiStereoCam(const std::vector<cv::Mat> leftCams,const std::vector<cv::Mat> rightCams,
                                        std::vector<cv::Mat> RT_pose_of_stereo_cams,
                                        cv::Mat &RT_mat_of_multi_stereo_cam_output,
                                        bool &match_success,
                                        int* pMatchedIndexID_output = nullptr
                                        );
    
    float retrieveSceneWithMultiMonoCam(const std::vector<cv::Mat> images,std::vector<cv::Mat> RT_pose_of_mono_cams,cv::Mat &RT_mat_of_multi_mono_cam_output,bool& match_success,int* pMatchedIndexID_output = nullptr);

    void debugVisualize();//visualize pointcloud and cam pose.

    void readImage(vector<string>& image_paths);

    void setImageVecPath(vector<string>& imageVec, int left);

    cv::Mat fetchImage(size_t index, int left);

    void displayFeatureMatches(cv::Mat curImage, vector<cv::KeyPoint> curKps,
                               cv::Mat oldImage, vector<cv::KeyPoint> oldKps,
                               std::vector<cv::DMatch> matches, size_t loop_index);

    void displayFeatureMatches(size_t loop_index, ptr_frameinfo& current_frame, std::vector<cv::DMatch> matches);

    inline vector<cv::Point3f> pts3dto3f(vector<cv::Point3d>& input_pts)
    {
        vector<cv::Point3f> result;
        cv::Point3f pts3f;
        for(auto& pts3d : input_pts)
        {
            pts3f.x = pts3d.x;
            pts3f.y = pts3d.y;
            pts3f.z = pts3d.z;
            result.emplace_back(pts3d.x, pts3d.y, pts3d.z);

//            cout<<"pts3dto3f: "<<result[result.size()]<<endl;
        }
        return result;
    }

    void publishPoseHistory();

    size_t LoopClosureDebugIndex = 0;


    inline Scene& getScene()
    {
        return this->original_scene;
    }

private:

    void _init_retriever();

    Scene original_scene;

    //LoopClosingManager* ploop_closing_manager_of_scene;

    shared_ptr<LoopClosingManager> ploop_closing_manager_of_scene = nullptr;

    //cv_helper* mpCv_helper = nullptr;

    shared_ptr<cv_helper> mpCv_helper = nullptr;

    cv::Mat mCurrentImage;

    vector<string> mVecLeftImagePath;
    vector<string> mVecRightImagePath;
};
#endif
