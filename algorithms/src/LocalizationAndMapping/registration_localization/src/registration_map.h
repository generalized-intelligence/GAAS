#ifndef REGISTRATION_MAP_H_FILE
#define REGISTRATION_MAP_H_FILE

#include "../../../CommonLibs/gaas_types/typedefs.h"
#include "../../../CommonLibs/gaas_location/gps_info.h"
#include "opencv2/core/persistence.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>

class RegistrationMap
{
private:
    MapCloudT::Ptr pMapCloud;
    bool map_ready = false;
    float x_min,y_min,z_min;
    float x_max,y_max,z_max;
public:
    GPSInfo map_gps_info;
    using Ptr = std::shared_ptr<RegistrationMap>;
    std::string getName();
    bool loadMapFromFile(const std::string& path);
    void initSubmapBuffer();
    void getXYZMinMax(float& xmin,float& ymin,float& zmin,float& xmax,float& ymax,float& zmax)
    {
        xmin = x_min;
        ymin = y_min;
        zmin = z_min;
        xmax = x_max;
        ymax = y_max;
        zmax = z_max;
    }
    inline MapCloudT::Ptr getMapCloud()
    {
        return pMapCloud;
    }
};

bool RegistrationMap::loadMapFromFile(const std::string &map_path)
{
    bool flag_map = false;
    bool flag_gps_config = false;

    MapCloudT::Ptr pFullMap(new MapCloudT);

    this->pMapCloud = MapCloudT::Ptr(new MapCloudT);
    pcl::io::loadPCDFile(map_path, *pFullMap);
    if(pFullMap->size()<=0)
    {
        LOG(ERROR)<<"In loadPCDMap(): Map empty!"<<endl;
        throw "Error!";
    }

    pcl::VoxelGrid<LidarPointT> sor;
    sor.setInputCloud(pFullMap);
    double DOWNSAMPLE_SIZE = -1.0;
    if(!ros::param::get("downsample_size",DOWNSAMPLE_SIZE))
    {
        LOG(ERROR)<<"downsample size not set in RegistrationMap::loadMapFromFile()!"<<endl;
        throw "error!";
    }
    LOG(INFO)<<"[registration_map.h] downsample size:"<<DOWNSAMPLE_SIZE<<endl;
    sor.setLeafSize(DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE);
    sor.filter(*this->pMapCloud);


    if(pMapCloud->size()>0)
    {
        LOG(INFO)<<"map pointcloud size:"<<pMapCloud->size()<<endl;
        flag_map = true;
    }



    MapPointT pt_min,pt_max;
    pcl::getMinMax3D(*pMapCloud,pt_min,pt_max);
    x_min = pt_min.x;
    y_min = pt_min.y;
    z_min = pt_min.z;

    x_max = pt_max.x;
    y_max = pt_max.y;
    z_max = pt_max.z;


    cv::FileStorage map_gps_config;
    map_gps_config.open(map_path+".yaml",cv::FileStorage::READ);
    if(!map_gps_config.isOpened())
    {
        LOG(ERROR)<<"Map config file not found!"<<endl;
        throw "Error";
    }
    map_gps_config["initial_longitude"]>>this->map_gps_info.longitude;
    map_gps_config["initial_latitude"]>>this->map_gps_info.latitude;
    map_gps_config["initial_altitude"]>>this->map_gps_info.altitude;
    map_gps_config["coordinate_mode"]>>this->map_gps_info.coordinate_mode;
    if(this->map_gps_info.coordinate_mode!="NWU")
    {
        LOG(ERROR)<<"Coordinate mode is not NWU; not supported yet!"<<endl;
        throw "Error";
    }
    LOG(INFO)<<"[registration_localization] map loaded!"<<endl;
    return flag_map;
}
#endif
