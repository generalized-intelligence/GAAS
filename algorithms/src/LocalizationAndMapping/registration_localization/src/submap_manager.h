#ifndef SUBMAP_MANAGER_H
#define SUBMAP_MANAGER_H

#include <unordered_map>
#include "registration_map.h"
//管理多张子地图加载 多张子地图共用一个坐标系 是同一个大地图的不同部分
struct Submap{
    float x_size_m,y_size_m;
    float x_center,y_center;
};


//缓存区
class LocalizationSubmapManager
{
private:
    RegistrationMap inner_fullmap;
    typedef std::tuple<int,int> partIndex;
    partIndex currentSubmapIndex;
    int submap_count_x,submap_count_y;
    int partIndexToIntIndex(const partIndex& i);
    std::unordered_map<int,Submap> index_to_submap;
    void createSubmapsFromMapFile(const std::string& map_path);
public:
    void assignToMap(const RegistrationMap& map);
    bool initAllSubmaps();
    bool needSwapBuffer();
    bool loadSubmapByXYZ();
};

bool LocalizationSubmapManager::needSwapBuffer(const Eigen::Matrix4f& pose)
{
    //判断高度，xy位置

    //区域分块，中心点---->不必切换地图的范围---->需要切换的范围---->出界
}




#endif
