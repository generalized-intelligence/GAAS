#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Geometry>

#include "Timer.h"
#include "typedefs.h"

bool loadPCDmap(MapCloudT::Ptr& pmap_cloud)
{
    string map_path;
    bool path_exist = ros::param::get("map_path",map_path);
    if(!path_exist)
    {
        LOG(ERROR)<<"Fatal error in map_publisher: get map_path failed!"<<endl;
        LOG(ERROR)<<"map_path:"<<map_path<<endl;
        exit(-1);
    }
    pmap_cloud = MapCloudT::Ptr(new MapCloudT);
    pcl::io::loadPCDFile(map_path, *pmap_cloud);
    if(pmap_cloud->size()>0)
    {
        LOG(INFO)<<"map pointcloud size:"<<pmap_cloud->size()<<endl;
        return true;
    }
    return false;
}
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("map_publishing_node");
    ros::init(argc,argv,"map_publishing_node");
    ros::NodeHandle nh;
    bool ndt_result_visualization = false;
    ros::param::get("ndt_result_visualization",ndt_result_visualization);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/gaas/visualization/localization/ndt_map",1);
    if(ndt_result_visualization)
    {
        MapCloudT::Ptr pmap=nullptr;
        if(!loadPCDmap(pmap))
        {
            LOG(ERROR)<<"Load map failed!"<<endl;
            return -1;
        }
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*pmap,map_msg);
        LOG(INFO)<<"map msg generated."<<endl;
        map_msg.header.frame_id="map";

        ros::Rate loop_rate(1);
        while(ros::ok())
        {
            //if(map_pub.getNumSubscribers()>1)
            if(true)
            {
                map_msg.header.stamp = ros::Time::now();
                map_pub.publish(map_msg);
                ros::spinOnce();
                LOG(INFO)<<"Map published."<<endl;
            }
            loop_rate.sleep();
        }
    }
    return 0;
}
