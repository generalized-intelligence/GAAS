#ifndef REGISTRATION_MAP_MANAGER_H
#define REGISTRATION_MAP_MANAGER_H


#include <glog/logging.h>
#include <ros/ros.h>
#include "registration_map.h"
#include <string>
#include <map>
#include <unordered_map>
#include <memory>
#include <mutex>
#include "../../../CommonLibs/gaas_timer/Timer.h"
#include "../../../CommonLibs/gaas_location/geometry.h"

//#include "submap_manager.h"
#include "pcl/filters/crop_box.h"



const bool USE_DYNAMIC_CROPBOX = true;
//const double CROP_HALF_RANGE = 80;
const double CROP_HALF_RANGE = 40;
class RegistrationMapManager
{
private:
    std::unordered_map<std::string,RegistrationMap::Ptr> map_name_to_map_ptr;
    std::string current_map_name;
    RegistrationMap::Ptr current_map;

//Only define them when USE_DYNAMIC_CROPBOX set.
    Eigen::Vector3d currentCropboxCenterPosition;
    MapCloudT::Ptr pMapCropBuffer = nullptr;
    std::mutex map_crop_buffer_mutex;


    void cropMapWithNewCenter(Eigen::Vector3d newCenter)
    {
        this->map_crop_buffer_mutex.lock();
        if(this->pMapCropBuffer == nullptr)
        {
            pMapCropBuffer = MapCloudT::Ptr(new MapCloudT);
        }
        pMapCropBuffer->clear();
        //change pMapCropBuffer;TODO.
        //auto pNewCropCloud = MapCloudT::Ptr(new MapCloudT);
        pcl::CropBox<MapPointT> box(false);
        box.setInputCloud(current_map->getMapCloud());

        float xmin,ymin,zmin,xmax,ymax,zmax;
        current_map->getXYZMinMax(xmin,ymin,zmin,xmax,ymax,zmax);
        Eigen::Vector4f min_pt (newCenter[0]-CROP_HALF_RANGE, newCenter[1]-CROP_HALF_RANGE,zmin, 1.0f);
        Eigen::Vector4f max_pt (newCenter[0]+CROP_HALF_RANGE, newCenter[1]+CROP_HALF_RANGE,zmax, 1.0f);
        box.setMin(min_pt);
        box.setMax(max_pt);
        MapCloudT mp_tmp;
        box.filter(mp_tmp);
        pcl::copyPointCloud(mp_tmp,*pMapCropBuffer);
        this->map_crop_buffer_mutex.unlock();
    }

//end of block

    bool loadAllMaps();
public:
    using Ptr=std::shared_ptr<RegistrationMapManager>;
    bool init(ros::NodeHandle& nh);
    void getMapByName();
    void selectMapByName(const std::string& map_name); //选择当前定位模块使用的地图
    //void getMapNameByGPSPosition();
    //void switchMapByGPSPosition();
    //std::shared_ptr<RegistrationMap> getCurrentMap();
    std::string getCurrentMapName();
    void addToMemBuffer(const std::string& name);
    RegistrationMap::Ptr getCurrentMap();
    void removeFromMemBuffer(const std::string& name);
    MapCloudT::Ptr getCurrentMapCloudBuffer();
    MapCloudT::Ptr getCurrentMapCloud(const Eigen::Matrix4f& position);
    MapCloudT::Ptr getCurrentMapCloud(const Eigen::Vector3d& position); // 获取地图点云块，切片，调度等细节实现隐藏在内部。
};
RegistrationMap::Ptr RegistrationMapManager::getCurrentMap()
{
    return this->current_map;
}
void RegistrationMapManager::selectMapByName(const std::string& map_name)
{
    LOG(INFO)<<"Switch to map "<<map_name<<endl;//TODO:....
    this->current_map_name = map_name;
    current_map = this->map_name_to_map_ptr.at(map_name);
}
bool RegistrationMapManager::loadAllMaps()
{
    std::string map_config_file_path;
    if(!ros::param::get("map_config_file_path",map_config_file_path))
    {
        LOG(ERROR)<<"[registration_localization] loadAllMaps() error: map config file param not found!"<<endl;
        throw "Error!";
    }
    cv::FileStorage fs;
    fs.open(map_config_file_path,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        LOG(ERROR)<<"[registration_localization] loadAllMaps() error: map config file invalid!"<<endl;
        throw "Error!";
    }
    cv::FileNode maps_node = fs["Maps"];
    cv::FileNodeIterator it = maps_node.begin(), it_end = maps_node.end(); // Go through the node
    for (; it != it_end; ++it)
    {//load all maps in memory:
        RegistrationMap::Ptr pMap(new RegistrationMap);
        std::string map_path_in;
        (*it)["path"]>>map_path_in;
        pMap->loadMapFromFile(map_path_in);
        std::string map_name;
        (*it)["name"]>>map_name;
        LOG(INFO)<<"[registration_localization] loaded map "<<map_name<<" successfully!"<<endl;
        this->map_name_to_map_ptr[map_name] = pMap;
    }
    return true;
}


std::string RegistrationMapManager::getCurrentMapName()
{
    return this->current_map_name;
}
bool RegistrationMapManager::init(ros::NodeHandle &nh)
{
    //load all maps
    loadAllMaps();
    //select for initial localization.
    std::string active_map_name;//选择地图
    if(!ros::param::get("active_map_name",active_map_name))
    {
        LOG(ERROR)<<"[registration_localization] active_map_name not found!"<<endl;
        throw "error";
    }
    if(this->map_name_to_map_ptr.count(active_map_name) == 0)
    {
        LOG(ERROR)<<"[registration_localization] active_map_name not loaded!"<<endl;
        throw "error";
    }
    selectMapByName(active_map_name);
    if(USE_DYNAMIC_CROPBOX)
    {
        this->currentCropboxCenterPosition<<0,0,0;
        cropMapWithNewCenter(this->currentCropboxCenterPosition);
    }
}
MapCloudT::Ptr RegistrationMapManager::getCurrentMapCloudBuffer()
{
    return this->pMapCropBuffer;
}
MapCloudT::Ptr RegistrationMapManager::getCurrentMapCloud(const Eigen::Matrix4f& position) // 获取地图点云块，切片，调度等细节实现隐藏在内部。
{
    Eigen::Vector3d v3d(position(0,3),position(1,3),position(2,3));
    return getCurrentMapCloud(v3d);
}
MapCloudT::Ptr RegistrationMapManager::getCurrentMapCloud(const Eigen::Vector3d& position) // 获取地图点云块，切片，调度等细节实现隐藏在内部。
{

    const double THRES_NEWCROP = 50;
    if(USE_DYNAMIC_CROPBOX)
    {
        if(getEuclideanDistance2D(this->currentCropboxCenterPosition,position)>THRES_NEWCROP) //calc 2d distance to determine if new map crop is needed.
        {
            this->cropMapWithNewCenter(position);
        }
    }
    return this->pMapCropBuffer;
}


#endif
