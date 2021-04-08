#include "AStarLib.h"
#include "../../obstacle_drivable_block_publisher/src/MapBlock.h"

#include <glog/logging.h>
#include "Timer.h"

//test astarlib by loading a map and do navigation.
class AStarTestEnv
{
public:
    MapCloudT::Ptr pmap_cloud = nullptr;
    MapBlock::Ptr pMb;
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    AStarCostMap ascm;
    int path_id;
    std::shared_ptr<ros::Publisher> pMapPub,pPathPub;

    void init(int argc,char** argv)
    {
        ros::init(argc,argv,"astar_debug_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        pMapPub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        *pMapPub = pNH->advertise<visualization_msgs::MarkerArray>("/gaas/navigation/offline_map_block_generator",1);
        pPathPub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        *pPathPub = pNH->advertise<visualization_msgs::Marker>("/gaas/navigation/offline_astar_visualization",1);
        string map_path;
        if(!ros::param::get("map_path",map_path))
        {
            LOG(ERROR)<<"Error:Map path not set!"<<endl;
            throw "Error!";
        }
        pmap_cloud = MapCloudT::Ptr(new MapCloudT);
        pcl::io::loadPCDFile(map_path,*pmap_cloud);
        if(pmap_cloud->size() == 0)
        {
            LOG(ERROR)<<"Error:map empty!"<<endl;
            throw "Error!";
        }
        pMb = MapBlock::Ptr(new MapBlock);
        pMb->initMapBlock(pmap_cloud);
        ScopeTimer t("Generate AStar Map");
        LOG(INFO)<<"generating a* map..."<<endl;
        ascm.initAStarMapByBlockMap(this->pMb);
    }
    void visualizeMap()
    {
        visualization_msgs::MarkerArray mks;
        pMb->visualizeToMarkerArray(mks);
        int i = 0;
        for(auto& u:mks.markers)
        {
            u.id = i++;
        }
        LOG(INFO)<<"publishing marker array with "<<mks.markers.size()<<" markers!"<<endl;
        pMapPub->publish(mks);
    }
    vector<TIndex> testAstar(int xi,int yi,int zi,int xf,int yf,int zf)
    {
        ScopeTimer t("Run A*");
        LOG(INFO)<<"Test A*..."<<endl;
        auto u = ascm.doAstar(xi,yi,zi,xf,yf,zf);
        LOG(INFO)<<"u.size"<<u.size()<<",path:"<<endl;
        for(auto v:u)
        {
            LOG(INFO)<<v[0]<<","<<v[1]<<","<<v[2]<<endl;
        }
        return u;
    }
    void visualizeAStar(vector<TIndex>& astar_path)
    {
        visualization_msgs::Marker lineList;
        //lineList.header.stamp = ros::Time::now();
        lineList.header.frame_id = "map";
        lineList.type = lineList.LINE_STRIP;
        lineList.ns = "visualize_astar_path";
        lineList.action = lineList.ADD;
        lineList.id = this->path_id++;
        lineList.color.r = 1.0;
        lineList.color.g = 1.0;
        lineList.color.b = 0.0;
        lineList.color.a = 1.0;
        lineList.scale.x = 0.3;
        //        for(int i=0;i<astar_path.size()-1;i++)
        //        {
        //            for(int j = 0;j<2;j++)
        //            {
        //                geometry_msgs::Point pt;
        //                auto index = astar_path.at(i+j);
        //                int x=index[0],y=index[1],z=index[2];
        //                pt.x = ascm.original_map->blockAt(x,y,z).cx;
        //                pt.y = ascm.original_map->blockAt(x,y,z).cy;
        //                pt.z = ascm.original_map->blockAt(x,y,z).cz;
        //                lineList.points.push_back(pt);
        //            }
        //        }
        lineList.colors.resize(astar_path.size());
        for(int i=0;i<astar_path.size();i++)
        {
            geometry_msgs::Point pt;
            auto index = astar_path.at(i);
            int x=index[0],y=index[1],z=index[2];
            pt.x = ascm.original_map->blockAt(x,y,z).cx;
            pt.y = ascm.original_map->blockAt(x,y,z).cy;
            pt.z = ascm.original_map->blockAt(x,y,z).cz;
            lineList.colors.at(i).r=1.0;
            lineList.colors.at(i).g=1.0;
            lineList.colors.at(i).b=0.0;
            lineList.colors.at(i).a=1.0;
            lineList.points.push_back(pt);
        }
        lineList.pose.orientation.w = 1;
        pPathPub->publish(lineList);
        LOG(INFO)<<"Visualizer published astar path!"<<endl;
        sleep(4);
    }
};

int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("astar_debug_node");
    AStarTestEnv aenv;
    aenv.init(argc,argv);
    LOG(INFO)<<"Waiting for rviz initializaion...5s left."<<endl;
    sleep(5);
    aenv.visualizeMap();

    int ix,iy,iz,fx,fy,fz;

    //test 1.
    aenv.ascm.original_map->getMapBlockIndexByXYZ(0,0,0,ix,iy,iz);
    aenv.ascm.original_map->getMapBlockIndexByXYZ(0,-30,0,fx,fy,fz);
    auto path = aenv.testAstar(ix,iy,iz,fx,fy,fz);
    aenv.visualizeAStar(path);

    aenv.ascm.original_map->getMapBlockIndexByXYZ(35,15,0,fx,fy,fz);
    path = aenv.testAstar(ix,iy,iz,fx,fy,fz);
    aenv.visualizeAStar(path);

//    aenv.visualizeAStar(path);
//    path = aenv.testAstar(0,0,5,0,5,7);
//    aenv.visualizeAStar(path);
//    path = aenv.testAstar(0,0,5,0,5,7);//test same path;
//    path = aenv.testAstar(0,0,5,0,58,7);
//    aenv.visualizeAStar(path);
    path = aenv.testAstar(0,0,5,2500,3,500);//test out of range condition.
    LOG(INFO)<<"test finished."<<endl;
    //aenv.visualizeMap();

    return 0;
}
