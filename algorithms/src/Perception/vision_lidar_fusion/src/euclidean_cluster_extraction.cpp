#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <glog/logging.h>


using std::string;
using std::vector;
using std::cout;
using std::endl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> LidarCloudT;
ros::NodeHandle* pNH = nullptr;


void doEuclideanSegment (const pcl::PCLPointCloud2::ConstPtr &cloud_msg, vector<pcl::PCLPointCloud2::Ptr> &output,
                         int min_cluster_size, int max_cluster_size, double cluster_tolerance)
{
    // Convert data to PointCloud<T>
    LidarCloudT::Ptr cloud_in (new LidarCloudT);
    pcl::fromPCLPointCloud2 (*cloud_msg, *cloud_in);

    // Estimate
    //TicToc tt;
    //tt.tic ();

    LOG(INFO)<<"In doEuclideanSegment()."<<endl;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);

    output.reserve (cluster_indices.size ());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
        extract.setInputCloud (cloud_msg);
        extract.setIndices (boost::make_shared<const pcl::PointIndices> (*it));
        pcl::PCLPointCloud2::Ptr out (new pcl::PCLPointCloud2);
        extract.filter (*out);
        output.push_back (out);
    }
    LOG(INFO)<<"doEuclideanSegment() Finished."<<endl;
}
void visualizeClusters(const vector<pcl::PCLPointCloud2::Ptr>& clusters)
{
    ;//TODO.
}
void callback(const pcl::PCLPointCloud2::ConstPtr &cloud_msg)
{
    vector<pcl::PCLPointCloud2::Ptr> output_clusters;
    const int min_cluster_size = 10;
    const int max_cluster_size = 20000000;
    const double cluster_tolerance = 0.4;
    doEuclideanSegment(cloud_msg,output_clusters,min_cluster_size,max_cluster_size,cluster_tolerance);
    visualizeClusters(output_clusters);
}

int main(int argc,char **argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO)<<"Start euclidean_cluster_fusion_node."<<endl;

    ros::init(argc,argv,"euclidean_cluster_fusion_node");
    ros::NodeHandle nh;
    pNH=&nh;

    return 0;
}
