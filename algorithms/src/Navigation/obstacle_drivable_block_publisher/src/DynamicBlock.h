#ifndef DYNAMIC_BLOCK_GRID_MAP_H
#define DYNAMIC_BLOCK_GRID_MAP_H
#include "MapBlock.h"
#include "Timer.h"
#include "gaas_msgs/GAASNavigationDynamicBlockGrid.h"
#include "gaas_msgs/GAASNavigationDynamicBlockMap.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>

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
    float tsdf = -1;    //invalid tsdf:-1.这个值只从外边设置，不使用就用-1填死。
    uint32_t status = 0;
    void toROSMsg(gaas_msgs::GAASNavigationDynamicBlockGrid& grid_msg)
    {
        grid_msg.cx = this->cx;
        grid_msg.cy = this->cy;
        grid_msg.cz = this->cz;
        grid_msg.obstacle_point_count = obstacle_point_count;
        grid_msg.status = status;
        grid_msg.tsdf = tsdf;
    }
    inline void toROSMarker(visualization_msgs::MarkerArray& mks,double grid_size, const ros::Time& stamp)//把自己放到marker array里面去.
    {
        if(this->tsdf>0.1)//只把障碍本体放进去，周围的不做可视化
        {
            return;
        }
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

    typedef std::unordered_map<int,DynamicGrid> MappingT;
    //typedef std::map<int,DynamicGrid> MappingT;

    MappingT indexToGrid;

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
            g.tsdf = 0;
            g.status = 0;//tsdf生成的点不认为被占用，交给下一个逻辑处理.
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
    void doCalcTSDF(const int max_tsdf_val)
    {
        ScopeTimer t("doCalcTSDF");
        //初始化查找表
        initDXYZToDist();
        //暂存集合
        auto prev_grids = this->indexToGrid;
        //遍历算tsdf
        t.watch("finished mapping copy:");
        LOG(INFO)<<"calc tsdf for:"<<prev_grids.size()<<" grids."<<endl;
        for(const auto& kv:prev_grids)
        {
            TIndex center_xyzindex = indexToTIndex(kv.first);
            for(int dx = -max_tsdf_val;dx<=max_tsdf_val;dx++)
            {
                for(int dy = -max_tsdf_val;dy<=max_tsdf_val;dy++)
                {
                    for(int dz = -max_tsdf_val;dz<=max_tsdf_val;dz++)
                    {
                        if(dx == 0&&dy==0&&dz ==0)
                        {
                            continue;
                        }
                        TIndex newtindex{center_xyzindex[0]+dx,center_xyzindex[1]+dy,center_xyzindex[2]+dz};
                        int index_new = indexFromTIndex(newtindex);
                        //float dist = calcLongDistByIndex(center_xyzindex,newtindex);
                        float dist = calcTSDFDistBydxyz(dx,dy,dz);
                        if(this->indexToGrid.count(index_new))
                        {//如果存在 检查并更新tsdf.
                            if(indexToGrid[index_new].tsdf>dist)
                            {
                                indexToGrid[index_new].tsdf = dist;
                            }
                        }
                        else
                        {
                            DynamicGrid g;
                            g.cx = this->x_min +  newtindex[0]*grid_size+grid_size*0.5;
                            g.cy = this->y_min +  newtindex[1]*grid_size+grid_size*0.5;
                            g.cz = this->z_min +  newtindex[2]*grid_size+grid_size*0.5;
                            g.map_index_x = newtindex[0];
                            g.map_index_y = newtindex[1];
                            g.map_index_z = newtindex[2];
                            g.obstacle_point_count = 0;
                            g.tsdf = dist;

                            this->indexToGrid[index_new] = g;
                        }
                    }
                }
            }

        }
    }

    void doConvolutionExpansion(const vector<TIndex>& kernel_shape)
    {
        //暂存县有障碍物的index集合
        //对每个暂存集合元素做扩张.
    }
    void doConvolutionExpansionToBlock(const TIndex& tIndex,const vector<TIndex>& kernel_shape)
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
private:
    inline float calcLongDistByIndex(const TIndex& i1,const TIndex& i2)//TODO:消除和AStarLib.h的重复代码.
    {
        float x2 = pow(i1[0]-i2[0],2);
        float y2 = pow(i1[1]-i2[1],2);
        float z2 = pow(i1[2]-i2[2],2);
        return sqrt(x2+y2+z2);
    }
    inline void initDXYZToDist()
    {
        LOG(INFO)<<"in initDXYZToDist()"<<endl;
        for(int x = 0;x<9;x++)
        {
            for(int y = 0;y<9;y++)
            {
                for(int z = 0;z<9;z++)
                {
                    dxyzToDist[x][y][z] = calcLongDistByIndex(TIndex{x-4,y-4,z-4},TIndex{0,0,0});
                }
            }
        }
    }
    float dxyzToDist[9][9][9];
    inline float calcTSDFDistBydxyz(int dx,int dy,int dz)
    {
        return dxyzToDist[dx+4][dy+4][dz+4];
    }


};

#endif
