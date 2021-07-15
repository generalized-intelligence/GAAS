#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include "gaas_msgs/GAASPerceptionObstacleCluster.h"
#include "gaas_msgs/GAASPerceptionObstacleClustersList.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <glog/logging.h>
#include "Timer.h"


using std::string;
using std::vector;
using std::cout;
using std::endl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> LidarCloudT;



typedef pcl::PointXYZRGB VisPointT;
typedef pcl::PointCloud<VisPointT> VisCloudT;

typedef pcl::PCLPointCloud2 MessageCloudT;

ros::NodeHandle* pNH = nullptr;
ros::Publisher* pVisPub = nullptr;
ros::Publisher* pObstaclesPub = nullptr;


//Load by ros params.
float EUCLIDEAN_DOWNSAMPLING_SIZE = 0.05;

int min_cluster_size = 200;
//const int max_cluster_size = 20000;
int max_cluster_size = 200000;
double cluster_tolerance = 0.25;


void doEuclideanSegment (const LidarCloudT::ConstPtr &cloud_in, vector<LidarCloudT::Ptr> &output,
                         int min_cluster_size, int max_cluster_size, double cluster_tolerance)
{
    // Convert data to PointCloud<T>
    //LidarCloudT::Ptr cloud_in (new LidarCloudT);
    //pcl::fromPCLPointCloud2 (*cloud_msg, *cloud_in);

    // Estimate
    //TicToc tt;
    //tt.tic ();

    LOG(INFO)<<"In doEuclideanSegment()."<<endl;
    ScopeTimer seg_timer("[EuclideanClusterExtraction] in doEuclideanSegment()");
    // Creating the KdTree object for the search method of the extraction
    if(cloud_in->empty())
    {
        LOG(INFO)<<"[euclidean_cluster_extraction] Empty point cloud input! Continue."<<endl;
        return;
    }

    LidarCloudT::Ptr downsampled_input(new LidarCloudT);
    pcl::VoxelGrid<PointT> sor;

    sor.setInputCloud(cloud_in);
    sor.setLeafSize(EUCLIDEAN_DOWNSAMPLING_SIZE, EUCLIDEAN_DOWNSAMPLING_SIZE, EUCLIDEAN_DOWNSAMPLING_SIZE);
    sor.filter(*downsampled_input);

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //tree->setInputCloud (cloud_in);
    tree->setInputCloud (downsampled_input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    //ec.setInputCloud (cloud_in);
    ec.setInputCloud(downsampled_input);
    ec.extract (cluster_indices);
    seg_timer.watch("[EuclideanClusterExtraction] extract clusters finished.");
    if(cluster_indices.size() == 0)
    {
        LOG(INFO)<<"[euclidean_cluster_extraction] No cluster found. Continue."<<endl;
        return;
    }
    output.reserve (cluster_indices.size());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
//        pcl::ExtractIndices<LidarCloudT> extract;
//        extract.setInputCloud (cloud_in);
//        extract.setIndices (boost::make_shared<const pcl::PointIndices> (*it));

        LidarCloudT::Ptr out (new LidarCloudT);
        for(auto i:it->indices)
        {
            out->points.push_back(downsampled_input->points.at(i));
        }
        out->height = 1;
        out->width = out->size();
        output.push_back(out);
        //LOG(INFO)<<"[EuclideanClusterExtraction]    cloud size:"<<out->size()<<endl;
    }
    LOG(INFO)<<"[EuclideanClusterExtraction] clusters count:"<<cluster_indices.size()<<endl;
    seg_timer.watch("[EuclideanClusterExtraction] generate cluster clouds finished.");
    LOG(INFO)<<"doEuclideanSegment() Finished."<<endl;
}
void visualizeClusters(const vector<LidarCloudT::Ptr>& clusters,const pcl::PCLPointCloud2::ConstPtr& original_msg)
{
    ScopeTimer vis_timer("[EuclideanClusterExtraction] in visualizeClusters()");
    vector<VisCloudT> vclouds;
    vclouds.reserve(clusters.size());
    for(int i = 0;i<clusters.size();i++)
    {
        vclouds.push_back(VisCloudT());
        unsigned char r,g,b;
        LidarCloudT::Ptr pCurrentCloud = clusters.at(i);
        r = i*1000%256;
        g = i*12429%256;
        b = pCurrentCloud->size()*43112%256;
        for(const PointT& pt:pCurrentCloud->points)
        {
            VisPointT vp;
            vp.x = pt.x;
            vp.y = pt.y;
            vp.z = pt.z;
            vp.r = r;
            vp.g = g;
            vp.b = b;
            vclouds.at(i).push_back(vp);
        }
        vclouds.at(i).height = 1;
        vclouds.at(i).width = vclouds.at(i).points.size();

    }
    vis_timer.watch("[EuclideanClusterExtraction] clusters copied.");
    //publish these clouds as one.
    MessageCloudT pc;
    VisCloudT vcloud_merged;

    for(const auto& vcloud:vclouds)
    {
        vcloud_merged+=vcloud;
    }
    pcl::toPCLPointCloud2(vcloud_merged,pc);
    pc.header.frame_id="lidar";
    pc.header.stamp=original_msg->header.stamp;
    pVisPub->publish(pc);
    vis_timer.watch("[EuclideanClusterExtraction] clusters published.");
}
void publishPerceptionObstaclesList(const vector<LidarCloudT::Ptr>& clusters,const pcl::PCLPointCloud2::ConstPtr& original_msg)
{
    gaas_msgs::GAASPerceptionObstacleClustersList obstacles_list;
    for(int i = 0;i<clusters.size();i++)
    {
        gaas_msgs::GAASPerceptionObstacleCluster new_cluster;
        new_cluster.obstacle_id = i;
        LidarCloudT::Ptr pCurrentCloud = clusters.at(i);
        pcl::toROSMsg(*pCurrentCloud,new_cluster.cloud);
        new_cluster.cloud.header.frame_id = "lidar";
        new_cluster.cloud.header.stamp.fromNSec(original_msg->header.stamp * 1000ull);
        obstacles_list.obstacles.push_back(new_cluster);
    }
    obstacles_list.header.frame_id = "lidar";
    obstacles_list.header.stamp.fromNSec(original_msg->header.stamp * 1000ull);//pcl stamp to ros
    pObstaclesPub->publish(obstacles_list);

}
void callback(const pcl::PCLPointCloud2::ConstPtr &cloud_msg)
{
    vector<LidarCloudT::Ptr> output_clusters;
    LidarCloudT::Ptr pCurrentCloud(new LidarCloudT);
    pcl::fromPCLPointCloud2(*cloud_msg,*pCurrentCloud);
    if(pCurrentCloud->empty())
    {
        LOG(WARNING)<<"[euclidean_cluster_extraction] Empty point cloud input! Continue."<<endl;
        return;
    }
    doEuclideanSegment(pCurrentCloud,output_clusters,min_cluster_size,max_cluster_size,cluster_tolerance);
    publishPerceptionObstaclesList(output_clusters,cloud_msg);
    visualizeClusters(output_clusters,cloud_msg);
}

int main(int argc,char **argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"Start euclidean_cluster_fusion_node."<<endl;

    ros::init(argc,argv,"euclidean_cluster_fusion_node");
    ros::NodeHandle nh;
    ros::param::get("euclidean_cluster_downsampling_size",EUCLIDEAN_DOWNSAMPLING_SIZE);
    ros::param::get("min_cluster_size",min_cluster_size);
    ros::param::get("max_cluster_size",max_cluster_size);
    ros::param::get("cluster_tolerance",cluster_tolerance);


    pNH=&nh;
    ros::Publisher vis_pub = nh.advertise<MessageCloudT>("/gaas/visualization/perception/euclidean_clusters_colored",10);
    pVisPub = &vis_pub;
    ros::Publisher perception_pub = nh.advertise<gaas_msgs::GAASPerceptionObstacleClustersList>("/gaas/perception/euclidean_original_clusters_list",10);
    pObstaclesPub = &perception_pub;
    ros::Subscriber sub = nh.subscribe<MessageCloudT>("/gaas/preprocessing/velodyne_downsampled",1,callback);

    ros::spin();
    return 0;
}
