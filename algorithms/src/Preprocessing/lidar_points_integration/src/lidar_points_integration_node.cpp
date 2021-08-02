#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <glog/logging.h>
#include "../../downsampling/src/Timer.h"
#include "../../ground_point_removal/src/ground_point_removal.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> LidarCloudT;
typedef sensor_msgs::PointCloud2 MessageCloudT;


class LidarPointsIntegrationManager
{
private:
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    ros::Publisher merged_cloud_pub;
    double velodyne_downsampling_size=0.1;
    double livox_downsampling_size=0.1;
    void syncVelodyneAndLivoxCallback(const MessageCloudT::ConstPtr velodyne_cloud_msg,
                                      const MessageCloudT::ConstPtr livox_cloud_msg)
    {
        LidarCloudT::Ptr pCurrentVelodyne(new LidarCloudT);
        pcl::fromROSMsg(*velodyne_cloud_msg,*pCurrentVelodyne);

        LidarCloudT::Ptr pCurrentLivox(new LidarCloudT);
        pcl::fromROSMsg(*livox_cloud_msg,*pCurrentLivox);

        LidarCloudT::Ptr pVelodyneDownsampled(new LidarCloudT),pLivoxDownsampled(new LidarCloudT);
        pcl::VoxelGrid<PointT> sor_velodyne;
        sor_velodyne.setInputCloud(pCurrentVelodyne);
        sor_velodyne.setLeafSize(velodyne_downsampling_size, velodyne_downsampling_size, velodyne_downsampling_size);
        sor_velodyne.filter(*pVelodyneDownsampled);

        pcl::VoxelGrid<PointT> sor_livox;
        sor_livox.setInputCloud(pCurrentLivox);
        sor_livox.setLeafSize(livox_downsampling_size, livox_downsampling_size, livox_downsampling_size);
        sor_livox.filter(*pLivoxDownsampled);


        LidarCloudT::Ptr pIntegrated(new LidarCloudT);
        *pIntegrated = (*pVelodyneDownsampled) + (*pLivoxDownsampled);
        LidarCloudT::Ptr pGroundRemoved(new LidarCloudT);
        {
            ScopeTimer t("[lidar_points_integration] Ground removal timer:");
            removeGround(pIntegrated,pGroundRemoved);
        }
        MessageCloudT pc;
        pcl::toROSMsg(*pGroundRemoved,pc);
//        MessageCloudT pc;
//        pcl::toROSMsg(*pIntegrated,pc);
        pc.header.frame_id = "lidar";
        pc.header.stamp = velodyne_cloud_msg->header.stamp;
        merged_cloud_pub.publish(pc);
    }
public:
    void initNode(int argc,char** argv)
    {
        ros::init(argc,argv,"px4_state_reporter_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        if(!(ros::param::get("velodyne_downsampling_size",velodyne_downsampling_size)&&
             ros::param::get("livox_downsampling_size",livox_downsampling_size)))
        {
            LOG(ERROR)<<"Param not set in lidar_points_integration_node, quit!"<<endl;
            throw "Error!";
        }
        //merged_cloud_pub = pNH->advertise<MessageCloudT>("/gaas/preprocessing/merged_cloud",1);
        merged_cloud_pub = pNH->advertise<MessageCloudT>("/gaas/preprocessing/velodyne_downsampled",1);

        message_filters::Subscriber<MessageCloudT> velodyne_sub(*pNH, "/velodyne_points2", 1);
        message_filters::Subscriber<MessageCloudT> livox_sub(*pNH, "/fake_livox/forward/horizon", 1);

        typedef message_filters::sync_policies::ApproximateTime<MessageCloudT, MessageCloudT> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, livox_sub);
        sync.registerCallback(boost::bind(&LidarPointsIntegrationManager::syncVelodyneAndLivoxCallback,this, _1, _2));
        ros::spin();
    }
};


int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"Start lidar_points_integration node."<<endl;
    LidarPointsIntegrationManager lpim;
    lpim.initNode(argc,argv);
    return 0;
}
