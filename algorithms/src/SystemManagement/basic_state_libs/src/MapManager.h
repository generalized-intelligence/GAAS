#ifndef GAAS_MAP_MANAGER_HEADER
#define GAAS_MAP_MANAGER_HEADER


#include "typedefs.h"


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include "../../../LocalizationAndMapping/lidar_localization/src/ndt_algo.h"

struct GAASMapT
{
    string map_name;
    MapCloudT::Ptr pCloud;
    MapGPSInfo map_gps_info; // from localization package.

    vector<LocationT> getTakeoffLocationAlternatives();
    vector<LocationT> getLandingLocationAlternatives();
    void serialize();
    void deserialize();
};

class MapManager
{
public:
    std::map<string,std::shared_ptr<GAASMapT> > name_to_map;
    bool activateMapByName(const string& map_name);//切换地图
    bool dynamicallyLoadMap(const string& map_name,std::shared_ptr<GAASMapT>& pNewMap);
    bool loadMaps();//加载地图
    //bool getMapByNameFromInternet();//网络动态加载
    string getCurrentMapName();//获取当前地图名称

    vector<LocationT> getCurrentMapTakeoffLocationAlternatives();//获取当前地图起飞用的地点.
    vector<LocationT> getCurrentMapLandingLocationAlternatives();//获取当前地图可用降落地点.


};

#endif
