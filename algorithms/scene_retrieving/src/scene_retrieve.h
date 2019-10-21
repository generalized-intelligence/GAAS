#ifndef SCENE_RETRIEVE_H
#define SCENE_RETRIEVE_H

#include <iostream>
#include <vector>

#include <glog/logging.h>
#include "DBoW3.h" // defines OrbVocabulary and OrbDatabase

#include <Eigen/Core>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d.hpp"

#include "LoopClosingManager.h"
#include "scene_frame_properties.h"
#include "serialization.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

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
    
    //inline 
    int getImageCount();
    
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




SceneFrame generateSceneFrameFromStereoImage(const cv::Mat &imgl, cv::Mat &imgr, const cv::Mat& RotationMat, const cv::Mat& TranslationMat, const cv::Mat& Q_mat, LoopClosingManager& lcm,bool& success)
{
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<1>"<<endl;
    std::vector<cv::KeyPoint> key_points2d_candidate;
    std::vector<cv::KeyPoint> key_points2d_final;
    std::vector<cv::Point3d> points3d;
    cv::Mat feature;
    //LOG(INFO)<<"generating temp LoopClosingManager in generateSceneFrameFromStereoImage()"<<endl;
    //LoopClosingManager lcm("./config/orbvoc.dbow3");

    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<2>"<<endl;
    //LOG(INFO)<<"temp LoopClosingManager extracting feature."<<endl;
    ptr_frameinfo pleft_image_info = lcm.extractFeature(imgl);
    ptr_frameinfo pright_image_info = lcm.extractFeature(imgr);
    
    cv::Mat feature_l,feature_r;
    feature_l = pleft_image_info->descriptors;
    feature_r = pright_image_info->descriptors;

    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<3>"<<endl;
    std::vector<int> good_2dmatches_index;

    LOG(INFO)<<"FlannBasedMatcher matching."<<endl;
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

    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<4>"<<endl;
    std::vector< cv::DMatch > good_matches;
    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= 4*min_dist && matches[i].distance< 10) // 3.0 too large;2.0 too large.
        {
            good_matches.push_back( matches[i]); 
        }
    }
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<5>"<<endl;
    
    std::vector<cv::Point2f> lk_input_keypoints, lk_output_keypoints;
    for(size_t i = 0; i < good_matches.size(); i++)
    {
        lk_input_keypoints.push_back(pleft_image_info->keypoints[good_matches[i].queryIdx].pt);//will check if LKFlow exist.
        good_2dmatches_index.push_back(good_matches[i].queryIdx);
    }
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<6>"<<endl;

    if (lk_input_keypoints.size()<=5)
    {
        SceneFrame failed;
        success = false;
        return failed;
    }


    std::vector<unsigned char> PyrLKResults;
    std::vector<float> optflow_err;
    LOG(INFO)<<"Calcing opt flow pyrlk."<<endl;
    cv::calcOpticalFlowPyrLK(imgl,
                            imgr,
                            lk_input_keypoints,
                            lk_output_keypoints,
                            PyrLKResults,
                            optflow_err);
    
    cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size())<<endl;
    cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size())<<endl;

    LOG(INFO)<<"lk_input_keypoints.size():"<<lk_input_keypoints.size()<<endl;
    LOG(INFO)<<"lk_output_keypoints.size():"<<lk_output_keypoints.size()<<endl;
    LOG(INFO)<<"PyrLKResults.size():"<<PyrLKResults.size()<<endl;
    LOG(INFO)<<"good_2d_index_matches.size()"<<good_2dmatches_index.size()<<endl; 
   
    if (lk_input_keypoints.size() < 5 || lk_output_keypoints.size() < 5 || PyrLKResults.size() < 5)
    {
        
         success = false;
         SceneFrame temp;
         return temp;
    }
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<7>"<<endl;
    
    std::vector<cv::Point2f> matched_points;
    std::vector<float> disparity_of_points;
    cv::Mat descriptors_reserved;

    LOG(INFO)<<"Descriptor size of left img: col,row:"<<pleft_image_info->descriptors.cols<<","<<pleft_image_info->descriptors.rows<<endl;
    for(int index = 0; index<lk_input_keypoints.size(); index++)
    {
        if(!PyrLKResults[index])
        {
            LOG(INFO)<<"pyrlk result index:"<<index<<"invalid.continue."<<endl;
            continue;
        }
        //if(PyrLKResults[index] == 1)
        else
        {
            LOG(INFO)<<"        adding pyrlk result index:"<<index<<"!"<<endl;
            LOG(INFO)<<"        result index in good_match:"<<good_2dmatches_index[index]<<"!"<<endl;
            matched_points.push_back(lk_input_keypoints[index]);
            disparity_of_points.push_back(lk_output_keypoints[index].x-lk_input_keypoints[index].x);
            
            LOG(INFO)<<"        in generateSceneFrameFromStereoImage() stage<1> split row"<<endl;
            cv::Mat desp_reserved = pleft_image_info->descriptors.rowRange(good_2dmatches_index[index], good_2dmatches_index[index]+1).clone().reshape(0,1);
            //cv::Mat desp_reserved = pleft_image_info->descriptors.row(good_2dmatches_index[index]).clone().reshape(0,1);
            LOG(INFO)<<"        in generateSceneFrameFromStereoImage() stage<2> add to mat."<<endl;
            descriptors_reserved.push_back(desp_reserved);//get this row out of mat.
            
            
            //push_back(pleft_image_info->descriptors[good_2dmatches_index[index]]);
        }
    }
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<8>"<<endl;

    std::vector<cv::Point3f> points3f;

    cv::reprojectImageTo3D(disparity_of_points, points3f, Q_mat);
    points3d.resize(points3f.size());
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<9>"<<endl;
    
    //do rotation and translation to points3d.
    for(int i = 0;i<points3f.size();i++)
    {
        cv::Mat point3d_temp(points3f[i]);
        
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
    LOG(INFO)<<"in generateSceneFrameFromStereoImage():step<10>"<<endl;
    cv::KeyPoint::convert(matched_points,key_points2d_final);
    
    LOG(INFO)<<"key_points2d_final size: "<<key_points2d_final.size()<<endl;
    LOG(INFO)<<"points3d size: "<<points3d.size()<<endl;
    success = true;  
    return std::make_tuple(key_points2d_final, points3d, descriptors_reserved, RotationMat, TranslationMat);
}



class SceneRetriever
{
public:
    
    SceneRetriever();
    
    SceneRetriever(const string& voc);
    
    //SceneRetriever(Scene& original_scene_input);
    SceneRetriever(const string& voc,const std::string& scene_file);

    SceneRetriever(const string& voc,const string& scene_file, const string config_file);

    float retrieveSceneWithScaleFromMonoImage(cv::Mat image_in_rect, cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,int* pMatchedIndexID_output = nullptr);

    float retrieveSceneFromStereoImage(cv::Mat& image_left_rect, cv::Mat& image_right_rect,
                                       cv::Mat& mavros_pose, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success,int* pMatchedIndexID_output = nullptr);

    // for SHAKESHAKE
    // Q is not used to recover 3D points now, but you can use this function now, and if you need my to use Q to recover 3D points I will modify related function.
    // input: image_left_rect, image_right_rect, Q_mat
    // output: outpuit: mavros_pose at the time of retrieving pose, recovered mat, bool and int.
    float retrieveSceneFromStereoImage(cv::Mat& image_left_rect, cv::Mat& image_right_rect,
                                       cv::Mat& mavros_pose, cv::Mat& RT_mat_of_stereo_cam_output, cv::Mat& Q_mat, bool& match_success,int* pMatchedIndexID_output = nullptr);

    inline int addFrameToScene(const std::vector<cv::KeyPoint>& points2d_in, const std::vector<cv::Point3d>points3d_in,const cv::Mat& point_desp_in, const cv::Mat R, const cv::Mat t)
    {
        LOG(INFO)<<"    content of desp in addFrameToScene():"<<point_desp_in<<endl;
        LOG(INFO)<<"in addFrameToScene():"<<endl;
        if(!this->ploop_closing_manager_of_scene )
        {
            LOG(ERROR)<<"Calling addFrameToScene() without initiated loop closing manager!"<<endl;
            return -1;
        }
        LOG(INFO)<<"in addFrameToScene() call addFrame():"<<endl;
        LOG(INFO)<<"size of point2d ,point3d:"<<points2d_in.size()<<","<<points3d_in.size()<<endl;
        this->original_scene.addFrame(points2d_in,points3d_in,point_desp_in,R,t);
        struct FrameInfo* pfr = new struct FrameInfo;
        LOG(INFO)<<"in addFrameToScene() query current index:"<<endl;
        LOG(INFO)<<"        index:"<<this->original_scene.getCurrentIndex()<<"obj size of p2d and desp:"<<this->original_scene.getP2D().size()<<","<<this->original_scene.point_desps.size()<<endl;
        pfr->keypoints = this->original_scene.getP2D().back();//this->original_scene.getP2D()[this->original_scene.getCurrentIndex()-1];
        pfr->descriptors = this->original_scene.getDespByIndex(original_scene.getCurrentIndex()-1);
        cout<<"pfr->keypoints size: "<<pfr->keypoints.size()<<endl;
        cout<<"pfr->descriptors size: "<<pfr->descriptors.size()<<endl;
        LOG(INFO)<<"in addFrameToScene() forming frame_info:"<<endl;
        ptr_frameinfo frame_info(pfr);
        LOG(INFO)<<"in addFrameToScene() adding KFrame:"<<endl;
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

    cv::Mat fetchImage(int index, int left);

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
        }
        return result;
    }

    void publishPoseHistory();

    size_t LoopClosureDebugIndex = 0;


    inline Scene& getScene()
    {
        return this->original_scene;
    }

    void MavrosPoseCallback(const geometry_msgs::PoseStamped& pose);

    inline cv::Mat PoseStampedToMat(const geometry_msgs::PoseStamped& pose)
    {
        Eigen::Quaterniond pose_q(pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z);

        Eigen::Matrix3d pose_R = pose_q.toRotationMatrix();

        cv::Mat mat_R;
        cv::eigen2cv(pose_R, mat_R);

        cv::Mat T = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);

        T.at<double>(0, 0) = mat_R.at<double>(0, 0);
        T.at<double>(0, 1) = mat_R.at<double>(0, 1);
        T.at<double>(0, 2) = mat_R.at<double>(0, 2);
        T.at<double>(1, 0) = mat_R.at<double>(1, 0);
        T.at<double>(1, 1) = mat_R.at<double>(1, 1);
        T.at<double>(1, 2) = mat_R.at<double>(1, 2);
        T.at<double>(2, 0) = mat_R.at<double>(2, 0);
        T.at<double>(2, 1) = mat_R.at<double>(2, 1);
        T.at<double>(2, 2) = mat_R.at<double>(2, 2);

        T.at<double>(0, 3) = pose.pose.position.x;
        T.at<double>(1, 3) = pose.pose.position.y;
        T.at<double>(2, 3) = pose.pose.position.z;
        T.at<double>(3, 3) = 1;

        return T;
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

    //ROS related
    ros::NodeHandle mNH;
    ros::Subscriber mMavrosSub;

    cv::Mat mCurMavrosPose;

private:

    float mThresFitnessScore = -1;

};
#endif
