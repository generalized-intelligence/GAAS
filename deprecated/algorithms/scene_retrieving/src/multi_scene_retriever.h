#ifndef MULTI_SCENE_RETRIEVER_H
#define MULTI_SCENE_RETRIEVER_H


#include "scene_retrieve.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

//Build a auto balanced kdtree to do quick retrieve of multi scene.
//Load multiple scenes in different place(maybe include the whole world) at once,and automaticly check the most related and closed ones.
//Will reserve all query interfaces of Scene Retriever. So just query the whole world just like querying a single scene.

#include "nlohmann/json.hpp"
using namespace nlohmann;

class MultiSceneNode
{

public:

    MultiSceneNode()
    {;}

    MultiSceneNode(bool load_scene = false)
    { //TODO:lazy download of pScene.
    }

    MultiSceneNode(shared_ptr<SceneRetriever> pSR,double lon,double lat)
    {
        this->pSceneRetriever=pSR;
        this->longitude = lon;
        this->latitude = lat;
    }

    void setSfMModelPath(const string& path);

    bool hasSfmModel()
    {
        return this->SfM_Model_path == "NO_PATH";
    }

public:
    shared_ptr<SceneRetriever> pSceneRetriever;
    double longitude;
    double latitude;
    std::string SfM_Model_path = std::string("NO_PATH");//load scene and visualize.
};

class MultiSceneRetriever//Support gps environment only.To do navigation indoor,just init a single SceneRetriever.
{

public:

    MultiSceneRetriever();

    void loadSceneInfoFromLocalFile(const string& scene_file_list_path,const string& voc_path)
    {
      //init this class itself.
	std::ifstream ifstr_json( scene_file_list_path);
	json json_obj;
	ifstr_json>>json_obj;
	auto scenes = json_obj["scenes"];
	json::iterator it;
	for(it = scenes.begin();it!=scenes.end();++it)
	{
	    auto scene_obj = it.value();
	    std::string scene_file_path(scene_obj["scene_path"].get<string>());   
	    shared_ptr<SceneRetriever> pSR( new SceneRetriever(voc_path,scene_file_path) );
	    double lon,lat;
	    lon = scene_obj["gps_longitude"].get<double>();
	    lat = scene_obj["gps_latitude"].get<double>();
	    shared_ptr<MultiSceneNode> pNode(new MultiSceneNode(pSR,lon,lat));
	    this->insertSceneIntoKDTree(pNode);//gps indexed!
	    if (scene_obj.find("sfm_model_path")!=scene_obj.end())
	    {
	        pNode->setSfMModelPath(scene_obj["sfm_model_path"].get<string>());
	    }
	}
    }
    void findNRelativeSceneByGPS(double gps_longitude,double gps_latitude,
                                vector<int> &output_scene_index,
                                int count = 10,double range_km = 5.0);
    void loadRelativeSceneInfoFromInternet(double gps_longitute,double gps_latitude,
                                      int count = 10,double range_km=5.0);
    void generate_visualization_graph();
    
    //for these methods,return best matched scene's id.
    //here we assert the return value of scene retriever's methods is matched points count.
    virtual int retrieveSceneWithScaleFromMonoImage(cv::Mat image_in_rect,
        cv::Mat cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,
        double img_lon,double img_lat,bool img_lon_lat_valid = false);
    
    virtual int retrieveSceneWithMultiStereoCam(const std::vector<cv::Mat> leftCams,const std::vector<cv::Mat> rightCams,
				      std::vector<cv::Mat> RT_pose_of_stereo_cams,
				      cv::Mat &RT_mat_of_multi_stereo_cam_output,
				      bool &match_success
				       );
    virtual int retrieveSceneWithMultiMonoCam(const std::vector<cv::Mat> images,std::vector<cv::Mat> RT_pose_of_mono_cams,cv::Mat &RT_mat_of_multi_mono_cam_output,bool& match_success);
    virtual int retrieveSceneFromStereoImage(cv::Mat image_left_rect, cv::Mat image_right_rect,
                                             cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output,
                                             bool& match_success,double img_lon,double img_lat,bool img_lon_lat_valid=false);

private:
    //void insertSceneIntoKDTree(double longi,double lati,shared_ptr<Scene> pScene);
    void insertSceneIntoKDTree(shared_ptr<MultiSceneNode> nodeptr);

private:
    int scene_index;
    map<int, shared_ptr<MultiSceneNode> > idToNodeMap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr gps_KDTree; // using pcl pointcloud as kdtree.

};



#endif
