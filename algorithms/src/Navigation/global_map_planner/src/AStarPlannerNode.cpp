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
#include "gaas_msgs/GAASNavigationDynamicBlockGrid.h"
#include "gaas_msgs/GAASNavigationDynamicBlockMap.h"

class AStarPlannerNode
{
public:
    MapCloudT::Ptr pmap_cloud = nullptr;
    MapBlock::Ptr pMb;
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    AStarCostMap ascm;
    int path_id=0;
    ros::ServiceServer service_server;
    std::shared_ptr<ros::Publisher> pMapPub,pPathPub;

    ros::Subscriber dynamic_map_subscriber;
    std::shared_ptr<DynamicMap> pDynamicMap;
    std::mutex dynamic_map_mutex;

    void initAStarPlannerNode(int argc,char** argv)
    {
        ros::init(argc,argv,"astar_planner_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        pMapPub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        *pMapPub = pNH->advertise<visualization_msgs::MarkerArray>("/gaas/visualization/navigation/map_blocks",1);
        pPathPub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        *pPathPub = pNH->advertise<visualization_msgs::Marker>("/gaas/visualization/navigation/astar_planned_path",1);
        dynamic_map_subscriber = pNH->subscribe<gaas_msgs::GAASNavigationDynamicBlockMap>("/gaas/navigation/dynamic_block_map",1,&AStarPlannerNode::dynamicMapCallback,this);
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
        LOG(INFO)<<"generating a* map..."<<endl;
        {
            ScopeTimer t("AStar Planning module load map");
            ascm.initAStarMapByBlockMap(this->pMb);
        }
        this->service_server = this->pNH->advertiseService("/gaas/navigation/global_planner/astar_planning",&AStarPlannerNode::planningServiceCallback,this);
        ros::spin();
    }
    void dynamicMapCallback(const gaas_msgs::GAASNavigationDynamicBlockMapConstPtr& dynamic_map_msg)
    {
        //3ms.
        ScopeTimer t("AStarPlannerNode::dynamicMapCallback Timer()");
        pDynamicMap = std::shared_ptr<DynamicMap>(new DynamicMap);
        pDynamicMap->fromROSMsg(*dynamic_map_msg);//获取dynamic_map_block_generator的动态地图.
        LOG(INFO)<<"in GP:dynamicMapCallback(): acquiring lock..."<<endl;
        dynamic_map_mutex.lock();
        this->ascm.setNewDynamicMap(pDynamicMap);//mutex set to avoid inconsistency.
        dynamic_map_mutex.unlock();
        LOG(INFO)<<"in GP:dynamicMapCallback(): lock released."<<endl;
    }
    bool planningServiceCallback(gaas_msgs::GAASGetAStarPath::Request& request,gaas_msgs::GAASGetAStarPath::Response& response)
    {
        ScopeTimer t_("planningServiceCallback() timer");
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
        t_.watch("before astar:");
        LOG(INFO)<<"in GlobalPlanner::planningServiceCallback(): acquiring lock..."<<endl;
        dynamic_map_mutex.lock();//protect astar session.
        vector<TIndex> output_path = ascm.doAstar(ix,iy,iz,fx,fy,fz);
        dynamic_map_mutex.unlock();
        LOG(INFO)<<"in GlobalPlanner::planningServiceCallback(): lock released."<<endl;
        t_.watch("after astar:");
        if(output_path.size()==0)
        {
            LOG(ERROR)<<"ERROR: in planningServiceCallback(): AStar algorithm failed!"<<endl;
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
                auto node = ascm.original_map->blockAt(n[0],n[1],n[2]);
                geometry_msgs::Point pt;
                pt.x = node.cx;
                pt.y = node.cy;
                pt.z = node.cz;
                response.path.path_nodes.push_back(pt);
            }
        }
        t_.watch("after response set:");
        visualizeAStar(output_path);
        t_.watch("after visualize,callback finished:");
        return true;
    }
    void visualizeAStar(vector<TIndex>& astar_path)
    {
        visualization_msgs::Marker lineList;
        //lineList.header.stamp = ros::Time::now();
        lineList.header.frame_id = "map";
        lineList.type = lineList.LINE_STRIP;
        lineList.ns = "visualize_astar_path";
        lineList.action = lineList.ADD;
        lineList.id = this->path_id;
        lineList.color.r = 1.0;
        lineList.color.g = 1.0;
        lineList.color.b = 0.0;
        lineList.color.a = 1.0;
        lineList.scale.x = 0.3;
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
    }
};
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("astar_planner_node");
    AStarPlannerNode node;
    node.initAStarPlannerNode(argc,argv);
    return 0;
}
