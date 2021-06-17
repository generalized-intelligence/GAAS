#ifndef ASTAR_LIB_H_FILE
#define ASTAR_LIB_H_FILE

#include <memory>
#include "../../obstacle_drivable_block_publisher/src/MapBlock.h"
#include "../../obstacle_drivable_block_publisher/src/DynamicBlock.h"
#include <glog/logging.h>

class AStarCostMap;
class AStarCostMapGrid;
class PathNode;



typedef std::array<int,3> TIndex;
TIndex getIndex(int x,int y,int z)
{
    TIndex retval;
    retval[0]=x;
    retval[1]=y;
    retval[2]=z;
    return retval;
}
class AStarCostMapGrid//不变量.
{
public:
    PathNode* pNode = nullptr;//初始化时放置.
    uint16_t obstacle_status=status_free;//是否有障碍物.
    static const int status_free = 0;
    static const int status_occupied = 1;
    static const int status_tsdf_danger = TSDF_DANGER;
    static const int MAX_TSDF_VAL = 4;//适应感知，改成4，原来是5.

    float TSDF=MAX_TSDF_VAL;//最近障碍物距离.0-MAX_TSDF_VAL,有障碍物为0;最近障碍在MAX_TSDF_VAL m外为MAX_TSDF_VAL.
};
class PathNode//变动的部分在这里.
{
public:
    AStarCostMapGrid* pGrid = nullptr;
    typedef std::shared_ptr<PathNode> Ptr;
    float cost = 0;
    float toNodeCost = 0;//到目前点所需要的cost.
    std::array<int,3> current_block_index;
    static const uint16_t not_visited = 0;
    static const uint16_t state_open = 1;
    static const uint16_t state_close = 2;
    uint16_t astar_status;//A*是否遍历过(从x个方向分别看;现在没想明白怎么实现).
    //之前的方向和现在的3d位置 一起编码进空间里.因此一共是4维, x,y,z,prev_velocity_direction.
    //根据这四个维度进行唯一查找.
    Ptr prev_node;//有向无环图,不必考虑循环引用.

    ~PathNode()
    {
        //LOG(INFO)<<"drop ptr"<<endl;
        pGrid->pNode = nullptr;
    }
};

bool isSmaller(const PathNode::Ptr& pN1,const PathNode::Ptr& pN2)
{
    return pN1->cost>pN2->cost;
}
struct PathNodePtrHeap
{
    vector<PathNode::Ptr> heap;
    PathNode::Ptr pop_heap(void)
    {
        std::pop_heap(heap.begin(),heap.end(),isSmaller);
        PathNode::Ptr node = heap.back();
        heap.pop_back();
        return node;
    }
    void push_heap(PathNode::Ptr pNew)
    {
        //make_heap(coll.begin(),coll.end());
        heap.push_back(pNew);
        std::push_heap(heap.begin(),heap.end(),isSmaller);
    }
    void clear()
    {
        this->heap.clear();
    }
    int getSize()
    {
        return this->heap.size();
    }
};



class AStarCostMap
{
public:
    int map_size_x,map_size_y,map_size_z;
    vector<AStarCostMapGrid> map_nodes_const;//不变量
    PathNodePtrHeap openList;
    MapBlock::Ptr original_map;
    std::shared_ptr<DynamicMap> p_dynamic_map=nullptr;

    PathNode::Ptr generatePathNode(int x,int y,int z)//设置新PathNode;PathNode的grid双向指针设计好.
    {
        //TODO.
        PathNode::Ptr newNode(new PathNode);
        newNode->current_block_index=getIndex(x,y,z);
        newNode->pGrid = &this->map_nodes_const.at(x*map_size_y*map_size_z+y*map_size_z+z);
        newNode->pGrid->pNode = newNode.get();

        return newNode;
    }
    void setNewDynamicMap(std::shared_ptr<DynamicMap> pNewDynamicMap)
    {
        this->p_dynamic_map = pNewDynamicMap;
    }
    bool checkXYZLegal(int x,int y,int z)
    {
        return x<map_size_x&&y<map_size_y&&z<map_size_z&&x>=0&&y>=0&&z>=0;
    }
    vector<PathNode::Ptr> getAlternatives(PathNode::Ptr pCurrent)//遍历周围所有点，找到没有越界,没有对应astar path node,可行驶没有障碍物的所有点.
    {
        int x,y,z;
        x = pCurrent->current_block_index[0];
        y = pCurrent->current_block_index[1];
        z = pCurrent->current_block_index[2];
        vector<TIndex> alternative_indices;
        vector<PathNode::Ptr> new_nodes;
        for(int x_d = -1;x_d<2;x_d++)//3x3范围所有临近元素.
        {
            for(int y_d = -1;y_d<2;y_d++)
            {
                for(int z_d = -1;z_d<2;z_d++)
                {
                    if((x_d == 0)&&(y_d==0)&&(z_d==0))
                    {
                        continue;
                    }
                    int cx,cy,cz;
                    cx = x+x_d;
                    cy = y+y_d;
                    cz = z+z_d;

                    if(checkXYZLegal(cx,cy,cz)&&map_nodes_const.at(cx*map_size_y*map_size_z+cy*map_size_z+cz).pNode==nullptr)
                    {
                        auto mapConstGrid = map_nodes_const.at(cx*map_size_y*map_size_z+cy*map_size_z+cz);
                        if(mapConstGrid.pNode == nullptr&&mapConstGrid.obstacle_status == mapConstGrid.status_free &&
                                (     !(p_dynamic_map->indexToGrid.count(cx*map_size_y*map_size_z+cy*map_size_z+cz)&&
                                           (p_dynamic_map->indexToGrid.at(cx*map_size_y*map_size_z+cy*map_size_z+cz).status!=0)//不存在感知模块看到的障碍节点.
                                       )
                                )
                          )
                        {
                            auto pNewNode = generatePathNode(cx,cy,cz);
                            pNewNode->prev_node = pCurrent;
                            new_nodes.push_back(pNewNode);
                        }
                    }
                }
            }
        }
        return new_nodes;
    }
    float calcCostOfPathAndNewAlternativeNode(PathNode::Ptr prev,PathNode::Ptr alternative,const TIndex& targetIndex)
    {
        float current_tsdf_value = 0;
        if(p_dynamic_map->indexToGrid.count(p_dynamic_map->indexFromTIndex(targetIndex)))//感知地图存在这个点。
        {
            float current_tsdf_value = std::min(alternative->pGrid->TSDF,
                                                p_dynamic_map->indexToGrid.at(p_dynamic_map->indexFromTIndex(targetIndex)).tsdf
                                                );
        }
        else
        {
            float current_tsdf_value = alternative->pGrid->TSDF;
        }
        float heuristic_cost_euclidean = calcLongDistByIndex(alternative->current_block_index,targetIndex);
        float heuristic_cost_euclidean_extra = 0;
        if(prev->current_block_index[2]!=alternative->current_block_index[2]&&prev->current_block_index[2]>=3)//非低空，高度变化
        {
            heuristic_cost_euclidean_extra=0.4;//惩罚高空起高度,因为激光雷达有死角.
        }
        else if(prev->current_block_index[2]!=alternative->current_block_index[2]&&prev->current_block_index[2]>=3)//低空，高度不变
        {
            heuristic_cost_euclidean_extra=0.5;//惩罚低空贴地.
        }
        heuristic_cost_euclidean += heuristic_cost_euclidean_extra;
        //float current_cost_tsdf = 0.4*(alternative->pGrid->MAX_TSDF_VAL - current_tsdf_value);//TODO:尝试这个参数是否应该调大.
        //float current_cost_tsdf = 2.2*(alternative->pGrid->MAX_TSDF_VAL - current_tsdf_value);//0.4可以,但是看着比较危险.
        float current_cost_tsdf = 4.4*(alternative->pGrid->MAX_TSDF_VAL - current_tsdf_value);//0.4可以,但是看着比较危险.
        float cost_current_step = calcShortDistByIndex(prev->current_block_index,alternative->current_block_index)+prev->toNodeCost+current_cost_tsdf;//如果经过currentNode就必须考虑这个tsdf.

        float final_cost = cost_current_step+heuristic_cost_euclidean;
        //LOG(INFO)<<"Node:"<<prev->current_block_index[0]<<","<<prev->current_block_index[1]<<","<<prev->current_block_index[2]<<":history+prev cost:"<<cost_current_step<<
        //           ";heuristic_cost_dist:"<<heuristic_cost_euclidean<<";cost_tsdf:"<<current_cost_tsdf<<";final cost:"<<final_cost<<endl;
        alternative->toNodeCost = cost_current_step;
        alternative->cost = final_cost;
        return final_cost;
        //先看上个节点前进方向,决定转向/继续前进的代价项.
        //再看到target的新距离.
        //TODO:可能考虑在一个阈值内到最近障碍物的距离,尤其是当飞机在当前前进方向上有障碍物时；以防止高速前进时减速不及时撞到障碍物.
    }
    int getNodePrevDirection(PathNode::Ptr pCurrent);
    vector<TIndex> doAstar(int xi,int yi,int zi,int xf,int yf,int zf)//xyz initial-final.
    {
        vector<TIndex> final_path;
        if(!(checkXYZLegal(xi,yi,zi)&&(checkXYZLegal(xf,yf,zf))))
        {
            LOG(ERROR)<<"ERROR:target or initial_point illegal!"<<endl;
            openList.clear();
            return final_path;
        }
        if(
	//		this->map_nodes_const.at(xi*map_size_y*map_size_z+yi*map_size_z+zi).obstacle_status!=AStarCostMapGrid::status_free
        //        ||
                this->map_nodes_const.at(xf*map_size_y*map_size_z+yf*map_size_z+zf).obstacle_status!=AStarCostMapGrid::status_free
                ||
                    (this->p_dynamic_map->indexToGrid.count(xf*map_size_y*map_size_z+yf*map_size_z+zf)&&
                     this->p_dynamic_map->indexToGrid.at(xf*map_size_y*map_size_z+yf*map_size_z+zf).status!=AStarCostMapGrid::status_free)//检查感知模块是否认为终点被占据.
                )
        {
            LOG(ERROR)<<"ERROR:target point occupied!"<<endl;
            openList.clear();
            return final_path;
        }

        bool flag_failed = false;
        PathNode::Ptr initial = generatePathNode(xi,yi,zi);
        initial->cost = 0;
        this->openList.push_heap(initial);
        if(getAlternatives(initial).size()==0)
        {
            LOG(WARNING)<<"Warning: initial point no valid neighbor!"<<endl;
        }
        auto current_pos = std::array<int,3>{xi,yi,zi};
        auto final_pos = std::array<int,3>{xf,yf,zf};
        const auto terminalIndex = getIndex(xf,yf,zf);
        PathNode::Ptr currMinCost;
        int loop_count = 0;
        while(true)
        {
            currMinCost = openList.pop_heap();
            //LOG(INFO)<<"current_pos:"<<currMinCost->current_block_index[0]<<";"<<currMinCost->current_block_index[1]<<";"<<currMinCost->current_block_index[2]<<endl;
            vector<PathNode::Ptr> alts = getAlternatives(currMinCost);
            for(auto alt:alts)
            {
                calcCostOfPathAndNewAlternativeNode(currMinCost,alt,terminalIndex);
                this->openList.push_heap(alt);
            }
            currMinCost->astar_status = currMinCost->state_close;//TODO.
            const int MAX_SIZE = 100000;//TODO.
            if(currMinCost->current_block_index==final_pos||openList.getSize()>=MAX_SIZE)
            {
                flag_failed = openList.getSize()>=MAX_SIZE;
                if(flag_failed)
                {
                    LOG(ERROR)<<"AStar failed due to overflow."<<endl;
                }
                break;
            }
            if(openList.getSize()==0&&currMinCost->current_block_index!=final_pos)
            {
                flag_failed = true;
                LOG(ERROR)<<"AStar failed due to no path(empty openList)."<<endl;
                break;
            }
            currMinCost->astar_status = currMinCost->state_close;
            loop_count++;
        }
        LOG(INFO)<<"AStar loop finished with "<<loop_count<<" times."<<endl;

        if(flag_failed)
        {
            LOG(ERROR)<<"ERROR:AStar failed by heap overflow or no valid path."<<endl;
            openList.clear();
            return final_path;
        }
        vector<PathNode::Ptr> final_path_reverse;
        while(currMinCost!=nullptr)
        {
            final_path_reverse.push_back(currMinCost);
            currMinCost = currMinCost->prev_node;
        }
        for(int i = final_path_reverse.size()-1;i>=0;i--)
        {
            final_path.push_back(final_path_reverse.at(i)->current_block_index);
        }
        openList.clear();
        return final_path;
        //智能指针自动实现 map结构里所有普通指针的恢复，什么都不用做
        //node::ptr全部重新生成,AStarCostMapGrid map_nodes_const不用动.
    }

//    void initializeConstMapNodeByOriginalMap(int x,int y,int z)//记录障碍物信息并计算tsdf
//    {
//        auto& node = this->map_nodes_const.at(x*map_size_y*map_size_z+y*map_size_z+z);
//        if(this->original_map->blockAt(x,y,z).isOccupied())
//        {
//            node.obstacle_status = node.status_occupied;
//        }
//        node.TSDF = node.MAX_TSDF_VAL;
//        //node.TSDF = 0;
//        auto currIndex = getIndex(x,y,z);
//        for(int dx = -node.MAX_TSDF_VAL;dx<=node.MAX_TSDF_VAL;dx++)
//        {
//            for(int dy = -node.MAX_TSDF_VAL;dy<=node.MAX_TSDF_VAL;dy++)
//            {
//                for(int dz = -node.MAX_TSDF_VAL;dz<=node.MAX_TSDF_VAL;dz++)
//                {
//                    int nx=x+dx,ny=y+dy,nz=z+dz;
//                    if(checkXYZLegal(nx,ny,nz))
//                    {
//                        if(this->original_map->blockAt(nx,ny,nz).isOccupied())
//                        {
//                            float dist = calcLongDistByIndex(currIndex,getIndex(nx,ny,nz));
//                            if(dist<node.TSDF)
//                            {
//                                node.TSDF = dist;//更新tsdf.
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        if(node.TSDF <=1)
//        {
//            node.obstacle_status |= node.status_tsdf_danger;
//        }
//    }
    void initializeConstMapNodeByOriginalMap(int x,int y,int z)
    {
        auto& node = this->map_nodes_const.at(x*map_size_y*map_size_z+y*map_size_z+z);
        auto currIndex = getIndex(x,y,z);
        if(this->original_map->blockAt(x,y,z).isOccupied())//如果这个格没有障碍就什么也不做.
        {
            node.obstacle_status = node.status_occupied;
            node.TSDF = 0;
            for(int dx = -node.MAX_TSDF_VAL;dx<=node.MAX_TSDF_VAL;dx++)
            {
                for(int dy = -node.MAX_TSDF_VAL;dy<=node.MAX_TSDF_VAL;dy++)
                {
                    for(int dz = -node.MAX_TSDF_VAL;dz<=node.MAX_TSDF_VAL;dz++)
                    {
                        int nx=x+dx,ny=y+dy,nz=z+dz;
                        if(checkXYZLegal(nx,ny,nz) &&!(dx==0&&dy==0&&dz==0))
                        {
                            float dist = calcLongDistByIndex(currIndex,getIndex(nx,ny,nz));
                            auto& neighbor_node = this->map_nodes_const.at(nx*map_size_y*map_size_z+ny*map_size_z+nz);
                            if(dist<neighbor_node.TSDF)
                            {
                                //LOG(INFO)<<"update neighbor tsdf to "<<dist<<"!"<<endl;
                                neighbor_node.TSDF = dist;//更新邻居的tsdf.
                            }
                            //if(neighbor_node.TSDF <=1.8)//max(1.414,1.732)
                            if(neighbor_node.TSDF <=1) //TSDF设置危险格的逻辑。
                            {
                                node.obstacle_status |= node.status_tsdf_danger;
                            }
                        }
                    }
                }
            }
        }
    }

    void initAStarMapByBlockMap(MapBlock::Ptr mapBlock)
    {
        this->original_map = mapBlock;
        map_size_x = mapBlock->map_size_x;
        map_size_y = mapBlock->map_size_y;
        map_size_z = mapBlock->map_size_z;

        //加载block map;
        map_nodes_const.resize(map_size_x*map_size_y*map_size_z);
        for (int i=0;i<map_nodes_const.size();i++)
        {
            int x = i/(map_size_y*map_size_z);
            int y = (i/map_size_z)%map_size_y;
            int z = i%map_size_z;
            initializeConstMapNodeByOriginalMap(x,y,z);//生成对应的AstarMapNode.
        }
        //计算类似tsdf的每个block到最近有障碍物block的距离.

        //init dynamic_map_pointer.
        this->p_dynamic_map = std::shared_ptr<DynamicMap>(new DynamicMap);//保证永远不是空指针。
        this->p_dynamic_map->map_size_x = mapBlock->map_size_x;
        this->p_dynamic_map->map_size_y = mapBlock->map_size_y;
        this->p_dynamic_map->map_size_z = mapBlock->map_size_z;
    }
    //void path_prone()//优化路径，根据记录的"tsdf" 计算是否能避免一些绕路过程.TODO.

private:
    inline float calcLongDistByIndex(const TIndex& i1,const TIndex& i2)
    {
        float x2 = pow(i1[0]-i2[0],2);
        float y2 = pow(i1[1]-i2[1],2);
        float z2 = pow(i1[2]-i2[2],2);
        return sqrt(x2+y2+z2);
    }
    inline float calcShortDistByIndex(const TIndex& i1,const TIndex& i2)//计算欧氏距离.
    {
        int d;
        d=abs(i1[0]-i2[0])+abs(i1[1]-i2[1])+abs(i1[2]-i2[2]);

        if(d==3)
        {
            return 1.732;//TODO:x1000并整数化避免浮点运算.
        }
        if(d==2)
        {
            return 1.414;
        }
        return 1.0;
    }
};



#endif
