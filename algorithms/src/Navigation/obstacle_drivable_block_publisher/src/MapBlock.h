#ifndef MAP_BLOCK_H_FILE
#define MAP_BLOCK_H_FILE
#include "typedefs.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "gaas_msgs/GAASNavigationMapBlockGrid.h"
#include "gaas_msgs/GAASNavigationBlockMap.h"
#include <glog/logging.h>

struct BasicBlock;
class MapBlock;
struct BasicBlock
{
    typedef std::shared_ptr<BasicBlock> Ptr;
    int grid_id;
    double cx,cy,cz; //center in map coordinate. //grid size defined in MapBlock.

    uint status=0;
    int map_point_count = 0;

    //check typedefs.h!
//    static const uint MAP_OCCUPIED= 1;
//    static const uint PERCEPTION_OCCUPIED = 2;
//    static const uint STATION_OCCUPIED = 4;
    inline bool isOccupied()
    {
        return status!=0;
    }
    inline void visualizeToMarkerArray(visualization_msgs::MarkerArray& mks,double grid_size, const ros::Time& stamp)//把自己放到marker array里面去.
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = "map";
        mk.header.stamp = stamp;
        mk.type = mk.CUBE;
        mk.action = mk.ADD;
        mk.pose.position.x = cx;// - 0.5*grid_size;
        mk.pose.position.y = cy;// - 0.5*grid_size;
        mk.pose.position.z = cz;// - 0.5*grid_size;
        mk.pose.orientation.w =1;
        mk.scale.x = grid_size;
        mk.scale.y = grid_size;
        mk.scale.z = grid_size;

        mk.color.a = this->status*0.6;
        mk.color.r = (this->status&MAP_OCCUPIED) *0.5 + (this->status&PERCEPTION_OCCUPIED)*0.25 + this->cx*0.05 + 0.1;
        mk.color.g = (this->status&PERCEPTION_OCCUPIED)*0.25 + this->cy*0.05 + 0.1;
        mk.color.b = 0.2+this->cz*0.2;
        if(this->status)
        {
            mks.markers.push_back(mk);
        }
    }
    void toROSMsg(gaas_msgs::GAASNavigationMapBlockGrid& grid)
    {
        grid.center.x = cx;
        grid.center.y = cy;
        grid.center.z = cz;
        grid.occupancy_status = status;
    }
    void fromROSMsg(const gaas_msgs::GAASNavigationMapBlockGrid& grid)
    {
        this->cx = grid.center.x;
        this->cy = grid.center.y;
        this->cz = grid.center.z;
        this->status = grid.occupancy_status;
        if(this->status&MAP_OCCUPIED||this->status&PERCEPTION_OCCUPIED)
        {
            this->map_point_count = 1000;//确保不会导致问题.
        }
    }
};

class MapBlock// 小格构造地图.与高精地图不同的是，只保留导航用的地图静态占用信息.
{
public:
    typedef std::shared_ptr<MapBlock> Ptr;
    typedef std::array<int,3> TIndex;
    vector<vector<vector<BasicBlock>>> block_map;
    double grid_size; //in metre.
    int map_size_x = 0,map_size_y = 0,map_size_z = 0;
    int xc,yc,zc;//(0,0,0)所占据的MapBlock block_map index.
    double x_min,y_min,z_min,x_max,y_max,z_max;

    MapCloudT::Ptr pMapCloud;

    //serializing.
    std::shared_ptr<gaas_msgs::GAASNavigationBlockMap> toROSMsg(const std_msgs::Header& header)
    {
        std::shared_ptr<gaas_msgs::GAASNavigationBlockMap> retval(new gaas_msgs::GAASNavigationBlockMap);
        retval->gridsize = this->grid_size;

        retval->x_count = map_size_x;
        retval->y_count = map_size_y;
        retval->z_count = map_size_z;

        retval->grids.resize(map_size_x*map_size_y*map_size_z);
        LOG(INFO)<<"in MapBlock::toROSMsg() Total grids:"<<map_size_x*map_size_y*map_size_z<<endl;
        for(int x_ = 0;x_<map_size_x;x_++)
        {
            for(int y_=0;y_<map_size_y;y_++)
            {
                for(int z_ = 0;z_<map_size_z;z_++)
                {
                    const int index_ = x_*map_size_y*map_size_z+y_*map_size_z+z_;
                    blockAt(x_,y_,z_).toROSMsg(retval->grids.at(index_));
                }
            }
        }
        retval->header = header;
        return retval;
    }
    static std::shared_ptr<MapBlock> fromROSMsg(const gaas_msgs::GAASNavigationBlockMap& map_msg)
    {
        std::shared_ptr<MapBlock> block(new MapBlock);
        block->grid_size = map_msg.gridsize;
        block->map_size_x = map_msg.x_count;
        block->map_size_y = map_msg.y_count;
        block->map_size_z = map_msg.z_count;
        block->block_map.resize(map_msg.x_count);
        for(auto& y__:block->block_map)
        {
            y__.resize(map_msg.y_count);
            for(auto&z__:y__)
            {
                z__.resize(map_msg.z_count);
            }
        }
        for(int x_ = 0;x_<map_msg.x_count;x_++)
        {
            for(int y_=0;y_<map_msg.y_count;y_++)
            {
                for(int z_ = 0;z_<map_msg.z_count;z_++)
                {
                    const int index_ = x_*map_msg.y_count*map_msg.z_count+y_*map_msg.z_count+z_;
                    block->blockAt(x_,y_,z_).fromROSMsg(map_msg.grids.at(index_));
                }
            }
        }
        return block;
    }

    inline BasicBlock& blockAt(int x,int y,int z)
    {
        return block_map.at(x).at(y).at(z);
    }
    inline BasicBlock& blockAt(const TIndex& index_)
    {
        return block_map.at(index_[0]).at(index_[1]).at(index_[2]);
    }
    inline TIndex getMapBlockIndexByXYZ(float x, float y,float z)
    {
        TIndex index_;
        index_[0] = (int) ((x-x_min)/grid_size);
        index_[1] = (int) ((y-y_min)/grid_size);
        index_[2] = (int) ((z-z_min)/grid_size);
        return index_;
    }
    void getMapBlockIndexByXYZ(float x, float y,float z,int& ix,int& iy,int& iz)
    {
        ix = (int) ((x-x_min)/grid_size);
        iy = (int) ((y-y_min)/grid_size);
        iz = (int) ((z-z_min)/grid_size);
        return;
    }
    void initMapBlock(MapCloudT::Ptr pMap)
    {
        loadHDMap(pMap);
        this->grid_size = 1.0;//set grid_size;
        //设置xyz min max;设置block_map的大小.
        bool size_correct = getXYZMinMaxSizeByPointCloud(*pMapCloud);
        if(!size_correct)
        {
            LOG(ERROR)<<"ERROR getXYZMinMaxSizeByPointCloud() invalid!"<<endl;
            throw "error!";
        }
        //循环设置每一个block.
        setBlockOccupancy(*pMapCloud);
    }
    void doConvolutionExpansion(const vector<TIndex>& vehicleOccupancyIndices)//根据飞机的尺寸，扩张障碍BlockMap.
    {//别忘了检查边界防止越界！
        LOG(WARNING)<<"doConvolutionExpansion() not implemented!"<<endl;
    }
    void loadFromDisk(string path);
    void writeToDisk(string path);
    bool getXYZMinMaxSizeByPointCloud(const MapCloudT& mapCloud);
    void setBlockOccupancy(const MapCloudT& mapCloud);
    void visualizeToMarkerArray(visualization_msgs::MarkerArray& mks)
    {
        for(int x_ = 0;x_<map_size_x;x_++)
        {
            for(int y_=0;y_<map_size_y;y_++)
            {
                for(int z_ = 0;z_<map_size_z;z_++)
                {
                    blockAt(x_,y_,z_).visualizeToMarkerArray(mks,grid_size,ros::Time::now());//TODO:检查timestamp设置策略.
                }
            }
        }
    }
    void eraseBasicBlocks() // 擦除所有block的tsdf和占据信息.
    {
        BasicBlock empty_block;
        int bb_id = 0;
        for(int x_ = 0;x_<map_size_x;x_++)
        {
            for(int y_=0;y_<map_size_y;y_++)
            {
                for(int z_ = 0;z_<map_size_z;z_++)
                {
                    auto& bb = blockAt(x_,y_,z_);
                    bb = empty_block;
                    bb.grid_id = bb_id;
                    bb_id+=1;
                    bb.cx = this->x_min + (x_)*grid_size+grid_size*0.5;
                    bb.cy = this->y_min + (y_)*grid_size+grid_size*0.5;
                    bb.cz = this->z_min + (z_)*grid_size+grid_size*0.5;
                }
            }
        }
    }
private:
    void loadHDMap(MapCloudT::Ptr pMap)
    {
        pMapCloud = pMap;
    }
};
void MapBlock::loadFromDisk(string path)
{
    throw "Not implemented!";//TODO.
}

bool MapBlock::getXYZMinMaxSizeByPointCloud(const MapCloudT& mapCloud)
{
    LOG(INFO)<<"in "<<__func__<<" stage 1."<<endl;
    if(mapCloud.size() == 0)
    {
        return false;
    }
    //step<1> 获取xyz max min.
    x_min=100000,y_min=100000,z_min=100000;
    x_max=-100000,y_max=-100000,z_max=-100000;
    for(const MapPointT& pt:mapCloud.points)
    {
        if(pt.x<x_min)
        {
            x_min = pt.x;
        }
        else if(pt.x>x_max)
        {
            x_max = pt.x;
        }
        if(pt.y<y_min)
        {
            y_min = pt.y;
        }
        else if(pt.y>y_max)
        {
            y_max = pt.y;
        }
        if(pt.z<z_min)
        {
            z_min = pt.z;
        }
        else if(pt.z>z_max)
        {
            z_max = pt.z;
        }
    }
    const double Z_MAX_MIN = 120; // 地图高度的最小值.
    if(z_max < Z_MAX_MIN)
    {
        z_max = Z_MAX_MIN;
    }
    //step<2> 获取grid size.
    LOG(INFO)<<"in "<<__func__<<" stage 2."<<endl;

    if(this->grid_size<0)
    {
        return false;
    }
    map_size_x = (int) ((x_max-x_min)/grid_size) +1;
    map_size_y = (int) ((y_max-y_min)/grid_size) +1;
    map_size_z = (int) ((z_max-z_min)/grid_size) +1;

    xc = (int) ((-1*x_min)/grid_size) ;
    yc = (int) ((-1*y_min)/grid_size) ;
    zc = (int) ((-1*z_min)/grid_size) ;
    LOG(INFO)<<"map_size xyz:"<<map_size_x<<","<<map_size_y<<","<<map_size_z<<endl;
    LOG(INFO)<<"map_center xyz:"<<xc<<","<<yc<<","<<zc<<endl;

    //step<3> 设置每个block的cx,cy,cz
    LOG(INFO)<<"in "<<__func__<<" stage 3."<<endl;

    this->block_map.resize(map_size_x);
    for(auto& u:block_map)
    {
        u.resize(map_size_y);
        for(auto& v:u)
        {
            v.resize(map_size_z);
        }
    }
    int bb_id = 0;
    for(int x_ = 0;x_ <this->map_size_x;x_++)
    {
        for(int y_ = 0;y_<this->map_size_y;y_++)
        {
            for(int z_ = 0;z_<this->map_size_z;z_++)
            {
                //设置grid_id;cx,cy,cz
                BasicBlock &bb = this->block_map.at(x_).at(y_).at(z_);
                bb.grid_id = bb_id;
                bb_id+=1;
                bb.cx = this->x_min + (x_)*grid_size+grid_size*0.5;
                bb.cy = this->y_min + (y_)*grid_size+grid_size*0.5;
                bb.cz = this->z_min + (z_)*grid_size+grid_size*0.5;
            }
        }
    }
    return true;
}
void MapBlock::setBlockOccupancy(const MapCloudT& mapCloud)
{
    for(const MapPointT&pt: mapCloud.points)
    {
        TIndex index_ = getMapBlockIndexByXYZ(pt.x,pt.y,pt.z);
        if(index_[0]<map_size_x&&index_[1]<map_size_y&&index_[2]<map_size_z&&index_[0]>=0&&index_[1]>=0&&index_[2]>=0)
        {
            BasicBlock& bb = this->blockAt(index_);
            bb.map_point_count++;
            if(bb.map_point_count>=3)//TODO: 阈值可设置
            {
                bb.status|=MAP_OCCUPIED;
            }
        }
    }
}
void MapBlock::writeToDisk(string path)
{
    throw "Not implemented!";//TODO.
}

#endif
