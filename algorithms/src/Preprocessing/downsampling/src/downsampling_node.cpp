#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <glog/logging.h>
#include "Timer.h"


using std::string;
using std::vector;
using std::cout;
using std::endl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> LidarCloudT;



typedef pcl::PCLPointCloud2 MessageCloudT;

ros::Publisher* pPub = nullptr;


//Load by ros params.
float PREPROCESSING_MODULE_DOWNSAMPLING_SIZE = 0.05;

void callback(const pcl::PCLPointCloud2::ConstPtr &cloud_msg)
{
    ScopeTimer downsampling_timer("down_sampling_callback");
    vector<LidarCloudT::Ptr> output_clusters;
    LidarCloudT::Ptr pCurrentCloud(new LidarCloudT);
    pcl::fromPCLPointCloud2(*cloud_msg,*pCurrentCloud);
    LidarCloudT::Ptr downsampled_input(new LidarCloudT);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(pCurrentCloud);
    sor.setLeafSize(PREPROCESSING_MODULE_DOWNSAMPLING_SIZE, PREPROCESSING_MODULE_DOWNSAMPLING_SIZE, PREPROCESSING_MODULE_DOWNSAMPLING_SIZE);
    sor.filter(*downsampled_input);

    pcl::PCLPointCloud2 pc;
    pcl::toPCLPointCloud2(*downsampled_input,pc);
    pc.header.frame_id = "lidar";
    pc.header.stamp = cloud_msg->header.stamp;
    pPub->publish(pc);
}

int main(int argc,char **argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"Start downsampling_node."<<endl;

    ros::init(argc,argv,"downsampling_node");
    ros::NodeHandle nh;
    ros::param::get("preprocessing_module_downsampling_size",PREPROCESSING_MODULE_DOWNSAMPLING_SIZE);

    ros::Publisher pub = nh.advertise<MessageCloudT>("/gaas/preprocessing/velodyne_downsampled",10);
    pPub = &pub;
    //ros::Subscriber sub = nh.subscribe<MessageCloudT>("/gaas/preprocessing/merged_cloud",1,callback); //2 lidars
    ros::Subscriber sub = nh.subscribe<MessageCloudT>("/velodyne_points2",1,callback); //one spinning lidar
    ros::spin();
    return 0;
}
