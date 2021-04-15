#ifndef DYNAMIC_BLOCK_GRID_MAP_H
#define DYNAMIC_BLOCK_GRID_MAP_H
#include "MapBlock.h"

#include "gaas_msgs/GAASNavigationDynamicBlockGrid.h"
#include "gaas_msgs/GAASNavigationDynamicBlockMap.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct DynamicGrid;
class DynamicMap;

struct DynamicGrid
{
    //map index.
    int map_index_x;
    int map_index_y;
    int map_index_z;

    //center xyz in map coordinate.
    double cx;
    double cy;
    double cz;

    int obstacle_point_count = 0;
    uint32_t status = 0;
    void toROSMsg(gaas_msgs::GAASNavigationDynamicBlockGrid& grid_msg)
    {
        grid_msg.cx = this->cx;
        grid_msg.cy = this->cy;
        grid_msg.cz = this->cz;
        grid_msg.obstacle_point_count = obstacle_point_count;
        grid_msg.status = status;
    }
    inline void toROSMarker(visualization_msgs::MarkerArray& mks,double grid_size, const ros::Time& stamp)//把自己放到marker array里面去.
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
};


class DynamicMap//动态地图既可以用来存储大地图，也可以处理小地图.
{
public:
    typedef MapBlock::TIndex TIndex;
    double grid_size; //in metre.
    int map_size_x = 0,map_size_y = 0,map_size_z = 0;
    int xc,yc,zc;//(0,0,0)所占据的MapBlock block_map index.
    double x_min,y_min,z_min,x_max,y_max,z_max;
    std::map<int,DynamicGrid> indexToGrid;

    void initDynamicMapBasicInfoWithMapBlock(const MapBlock& mb)//只初始化尺寸和基本属性，不记录网格.
    {
        map_size_x = mb.map_size_x;
        map_size_y = mb.map_size_y;
        map_size_z = mb.map_size_z;
        xc = mb.xc;
        yc = mb.yc;
        zc = mb.zc;
        grid_size = mb.grid_size;
        LOG(INFO)<<"mb.xyz min:"<<mb.x_min<<";"<<mb.y_min<<";"<<mb.z_min<<endl;

        x_min = mb.x_min;
        y_min = mb.y_min;
        z_min = mb.z_min;
        x_max = mb.z_max;
        y_max = mb.y_max;
        z_max = mb.z_max;
    }
    inline TIndex indexToTIndex(int index)
    {
        TIndex u;
        u[0] = index/(map_size_y*map_size_z);
        u[1] = index%(map_size_y*map_size_z)/map_size_z;
        u[2] = index%map_size_z;
        return u;
    }
    inline bool checkTIndexLegal(const TIndex& i)
    {
        return (i[0]>=0&&i[1]>=0&&i[2]>=0 && i[0]<map_size_x&&i[1]<map_size_y&&i[2]<map_size_z);
    }
    inline int indexFromTIndex(const TIndex& index)
    {
        return index[0]*map_size_y*map_size_z + index[1]*map_size_z+index[2];
    }
    inline TIndex queryTIndexFromxyz(double x,double y,double z)
    {
        TIndex index_;
        index_[0] = (int) ((x-x_min)/grid_size);
        index_[1] = (int) ((y-y_min)/grid_size);
        index_[2] = (int) ((z-z_min)/grid_size);
        return index_;
    }
    inline void addOrUpdateNewBlockByPointPosition(double x,double y,double z,const uint& occupied_status=PERCEPTION_OCCUPIED)//根据点云坐标尝试更新点云.
    //occupied status也可以是MAP_OCCUPIED.
    {
        TIndex pt_tindex = queryTIndexFromxyz(x,y,z);
        if(!checkTIndexLegal(pt_tindex))
        {//障碍物超出地图边界，不作任何操作.
            //LOG(INFO)<<"Index: "<<pt_tindex[0]<<","<<pt_tindex[1]<<","<<pt_tindex[2]<<" overflow!"<<endl;
            return;
        }
        int pt_index = indexFromTIndex(pt_tindex);
        if(indexToGrid.count(pt_index))
        {
            auto& g = indexToGrid.at(pt_index);
            g.obstacle_point_count++;
            if(g.obstacle_point_count>=1)//TODO: set a threshold in launch file.
            {
                //LOG(INFO)<<"set: "<<pt_tindex[0]<<","<<pt_tindex[1]<<","<<pt_tindex[2]<<" occupied!"<<endl;
                g.status|=occupied_status;
            }
        }
        else
        {
            DynamicGrid g;
            g.cx = this->x_min +  pt_tindex[0]*grid_size+grid_size*0.5;
            g.cy = this->y_min +  pt_tindex[1]*grid_size+grid_size*0.5;
            g.cz = this->z_min +  pt_tindex[2]*grid_size+grid_size*0.5;
            g.map_index_x = pt_tindex[0];
            g.map_index_y = pt_tindex[1];
            g.map_index_z = pt_tindex[2];
            g.obstacle_point_count = 1;
            g.status|=occupied_status;//TODO:Only when thres = 1.
            indexToGrid[pt_index] = g;
        }
    }
    void addNewPointCloud(LidarCloudT& cloud)//默认是map坐标系下.
    {
        for(const auto& pt:cloud.points)
        {
            addOrUpdateNewBlockByPointPosition(pt.x,pt.y,pt.z);
        }
    }
    void doConvolutionExpansionToBlock(const TIndex& tIndex)
    {
    }
    bool checkGridValid(int map_ix,int map_iy,int map_iz)//根据地图grid index查询网格在dynamicMap可用状态.
    {
        ;
    }
    std::shared_ptr<gaas_msgs::GAASNavigationDynamicBlockMap> toROSMsg(const std_msgs::Header& header)
    {
        std::shared_ptr<gaas_msgs::GAASNavigationDynamicBlockMap> pMap(new gaas_msgs::GAASNavigationDynamicBlockMap);
        pMap->header = header;
        pMap->grid_size = this->grid_size;
        pMap->map_size_x = this->map_size_x;
        pMap->map_size_y = this->map_size_y;
        pMap->map_size_z = this->map_size_z;
        pMap->xc = this->xc;
        pMap->yc = this->yc;
        pMap->zc = this->zc;

        pMap->x_min = this->x_min;
        pMap->x_max = this->x_max;

        pMap->y_min = this->y_min;
        pMap->y_max = this->y_max;

        pMap->z_min = this->z_min;
        pMap->z_max = this->z_max;
        for(auto& kv:this->indexToGrid)
        {
            gaas_msgs::GAASNavigationDynamicBlockGrid grid;
            TIndex tindex = indexToTIndex(kv.first);
            grid.map_index_x = tindex[0];
            grid.map_index_y = tindex[1];
            grid.map_index_z = tindex[2];
            kv.second.toROSMsg(grid);
            pMap->map_grids.push_back(grid);
        }
        return pMap;
    }
    void fromROSMsg()
    {
        ;
    }
    void toROSMarkers(visualization_msgs::MarkerArray& mks)
    {
        ros::Time stamp = ros::Time::now();
        int count = 0;
        for(auto& kv:this->indexToGrid)
        {
            DynamicGrid& g = kv.second;
            if(g.status)
            {
                g.toROSMarker(mks,grid_size,stamp);//TODO:检查timestamp设置策略.
                mks.markers.back().id = count;
                count++;
            }
        }

    }


};

#endif
