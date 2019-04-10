#ifndef MULTI_SCENE_RETRIEVER_H
#define MULTI_SCENE_RETRIEVER_H

#include "scene_retrieve.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

//Build a auto balanced kdtree to do quick retrieve of multi scene.
//Load multiple scenes in different place(maybe include the whole world) at once,and automaticly check the most related and closed ones.
//Will reserve all query interfaces of Scene Retriever.So just query the whole world just like querying a single scene.


class MultiSceneNode();

class MultiSceneRetriever
{
public:
    MultiSceneRetriever();
    void loadSceneInfoFromLocalFolder()
    {//init this class itself.
    }
    void findNRelativeSceneByGPS(double gps_longitude,double gps_latitude,
                                 int count = 10,double range_km = 5);
    void loadRelativeSceneInfoFromInternet(double gps_longitute,double gps_latitude,
                                      int count = 10,double range_km=5);

    virtual int retrieveSceneWithScaleFromMonoImage(const cv::Mat image_in_rect,
        const cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,
        double img_lon,double img_lat,bool img_lon_lat_valid = false);
    
    virtual int retrieveSceneWithMultiStereoCam(const std::vector<cv::Mat> leftCams,const std::vector<cv::Mat> rightCams,
				      std::vector<cv::Mat> RT_pose_of_stereo_cams,
				      cv::Mat &RT_mat_of_multi_stereo_cam_output,
				      bool &match_success
				       );
    virtual int retrieveSceneWithMultiMonoCam(const std::vector<cv::Mat> images,std::vector<cv::Mat> RT_pose_of_mono_cams,cv::Mat &RT_mat_of_multi_mono_cam_output,bool& match_success);
    virtual int retrieveSceneFromStereoImage(const cv::Mat image_left_rect, const cv::Mat image_right_rect, const cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success);
private:
    void insertSceneIntoKDTree(double longi,double lati,shared_ptr<Scene> pScene);
    void insertSceneIntoKDTree(shared_ptr<MultiSceneNode> nodeptr)
private:
    int scene_index;
    map<int,shared_ptr<MultiSceneNode> > idToNodeMap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr gps_KDTree; // using pcl pointcloud as kdtree.

};
class MultiSceneNode
{
public:
    MultiSceneNode()
    {;}
    MultiSceneNode(bool load_scene = False)
    { //TODO:lazy download of pScene.
    }
public:
    shared_ptr<Scene> pScene;
    double longitude;
    double latitude;
};


#endif
