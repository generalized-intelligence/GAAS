#include "AStarLib.h"
#include "../../obstacle_drivable_block_publisher/src/MapBlock.h"

#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <glog/logging.h>
#include "Timer.h"

#include "gaas_msgs/GAASNavigationPath.h"
#include "gaas_msgs/GAASGetAStarPath.h"

class AStarPlannerNode
{
public:
    MapCloudT::Ptr pmap_cloud = nullptr;
    MapBlock::Ptr pMb;
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    AStarCostMap ascm;
    int path_id;
    ros::ServiceServer service_server;
    //std::shared_ptr<ros::Publisher> pMapPub,pPathPub;
    void initAStarPlannerNode(int argc,char** argv)
    {
        ros::init(argc,argv,"astar_planner_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        //pMapPub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        //*pMapPub = pNH->advertise<visualization_msgs::MarkerArray>("/gaas/navigation/offline_map_block_generator",1);
        //pPathPub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        //*pPathPub = pNH->advertise<visualization_msgs::Marker>("/gaas/navigation/offline_astar_visualization",1);
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
        this->service_server = this->pNH->advertiseService("/gaas/navigation/global_planner/astar_planning",&AStarPlannerNode::planningServiceCallback,this);
        ros::spin();
    }
    bool planningServiceCallback(gaas_msgs::GAASGetAStarPath::Request& request,gaas_msgs::GAASGetAStarPath::Response& response)
    {
        if(request.header.frame_id!="map")
        {
            LOG(ERROR)<<"ERROR: in planningServiceCallback():Request not in map coordinate!"<<endl;
            return false;
        }
        int ix,iy,iz,fx,fy,fz;//initial and final xyz.
        auto& i_ = request.current_point.point;
        auto& f_ = request.target_point.point;

        ascm.original_map->getMapBlockIndexByXYZ(i_.x,i_.y,i_.z,ix,iy,iz);//map的初始位置
        ascm.original_map->getMapBlockIndexByXYZ(f_.x,f_.y,f_.z,fx,fy,fz);//map的目标

        vector<TIndex> output_path = ascm.doAstar(ix,iy,iz,fx,fy,fz);
        if(output_path.size()==0)
        {
            response.success=false;
        }
        else
        {
            response.success=true;
            response.path.header.frame_id = "map";
            response.path.header.stamp = ros::Time::now();
            response.path.path_nodes.clear();
            for(auto n:output_path)
            {
                geometry_msgs::Point pt;
                pt.x = n[0];
                pt.y = n[1];
                pt.z = n[2];
                response.path.path_nodes.push_back(pt);
            }
        }
        return true;
    }
};
 bool planningServiceCallback222(gaas_msgs::GAASGetAStarPath::Request& request,gaas_msgs::GAASGetAStarPath::Response& response)
 {
     return true;
 }
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("astar_debug_node");
    AStarPlannerNode node;
    node.initAStarPlannerNode(argc,argv);
    return 0;
}
