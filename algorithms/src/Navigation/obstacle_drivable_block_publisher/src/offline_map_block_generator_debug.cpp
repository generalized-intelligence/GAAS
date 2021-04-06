#include <pcl/io/pcd_io.h>

#include "typedefs.h"
#include "MapBlock.h"


class MapStaticBlockGenerator
{
public:
    MapCloudT::Ptr pmap_cloud = nullptr;
    MapBlock mb;
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;

    void initMapBlockGeneratorNode(int argc,char** argv)
    {
        ros::init(argc,argv,"offline_map_block_generator");
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
    }
    void generateBlocksFromMap()
    {
        mb.initMapBlock(pmap_cloud);
        ros::Publisher pub = pNH->advertise<visualization_msgs::MarkerArray>("/gaas/navigation/offline_map_block_generator",1);
        visualization_msgs::MarkerArray mks;
        this->mb.visualizeToMarkerArray(mks);
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
    void saveToDisk(string output_path)
    {
        LOG(ERROR)<<"saveToDist() not implemented!"<<endl;
        throw "error";
    }
};




int main(int argc,char** argv)
{
    MapStaticBlockGenerator msb;
    msb.initMapBlockGeneratorNode(argc,argv);
    msb.generateBlocksFromMap();
    //msg.saveToDisk(output_path);
    return 0;
}
