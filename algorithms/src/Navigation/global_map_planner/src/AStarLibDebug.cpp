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

    void init(int argc,char** argv)
    {
        ros::init(argc,argv,"astar_debug_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
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
        LOG(INFO)<<"genearating a* map..."<<endl;
        ascm.initAStarMapByBlockMap(this->pMb);
    }
    void visualizeMap()
    {
        ros::Publisher pub = pNH->advertise<visualization_msgs::MarkerArray>("/gaas/navigation/offline_map_block_generator",1);
        visualization_msgs::MarkerArray mks;
        pMb->visualizeToMarkerArray(mks);
        int i = 0;
        for(auto& u:mks.markers)
        {
            u.id = i++;
        }
        while(ros::ok())
        {
            LOG(INFO)<<"publishing marker array with "<<mks.markers.size()<<" markers!"<<endl;
            pub.publish(mks);
            sleep(4);
        }
    }
    void testAstar(int xi,int yi,int zi,int xf,int yf,int zf)
    {
        ScopeTimer t("Run A*");
        LOG(INFO)<<"Test A*..."<<endl;
        auto u = ascm.doAstar(xi,yi,zi,xf,yf,zf);
        LOG(INFO)<<"A* finished."<<endl;
        LOG(INFO)<<"u.size"<<u.size()<<",path:"<<endl;
        for(auto v:u)
        {
            LOG(INFO)<<v[0]<<","<<v[1]<<","<<v[2]<<endl;
        }
    }
};

int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("astar_debug_node");
    AStarTestEnv aenv;
    aenv.init(argc,argv);
    aenv.testAstar(0,0,2,0,5,7);
    aenv.testAstar(0,0,3,25,3,5);
    //aenv.visualizeMap();

    return 0;
}
