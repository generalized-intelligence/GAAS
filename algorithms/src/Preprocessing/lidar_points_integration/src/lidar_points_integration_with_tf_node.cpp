#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

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


struct LidarInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string name;
    std::string topic_name;
    std::string lidar_type;
    cv::Mat extrinsic_cv;
    Eigen::Matrix4f extrinsics_eigen;
    void cv2eigen()
    {
        for(int x = 0;x<4;x++)
        {
            for(int y = 0;y<4;y++)
            {
                extrinsics_eigen(x,y) = extrinsic_cv.at<float>(y,x);
            }
        }
    }
};

class LidarPointsIntegrationManager
{
private:
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    ros::Publisher merged_cloud_pub;
    double velodyne_downsampling_size=0.1;
    double livox_downsampling_size=0.1;
    void sync2VelodyneCallback(const MessageCloudT::ConstPtr cloud_msg1,
                                      const MessageCloudT::ConstPtr cloud_msg2)
    {
        LidarCloudT::Ptr pCurrentCloud1(new LidarCloudT);
        pcl::fromROSMsg(*cloud_msg1,*pCurrentCloud1);

        LidarCloudT::Ptr pCurrentCloud2(new LidarCloudT);
        pcl::fromROSMsg(*cloud_msg2,*pCurrentCloud2);

        LidarCloudT::Ptr pDownsampled1(new LidarCloudT),pDownsampled2(new LidarCloudT);
        pcl::VoxelGrid<PointT> sor_velodyne;
        sor_velodyne.setInputCloud(pCurrentCloud1);
        sor_velodyne.setLeafSize(velodyne_downsampling_size, velodyne_downsampling_size, velodyne_downsampling_size);
        sor_velodyne.filter(*pDownsampled1);

        pcl::VoxelGrid<PointT> sor_livox;
        sor_livox.setInputCloud(pCurrentCloud2);
        sor_livox.setLeafSize(livox_downsampling_size, livox_downsampling_size, livox_downsampling_size);
        sor_livox.filter(*pDownsampled2);

        LidarCloudT::Ptr pTransformed1(new LidarCloudT),pTransformed2(new LidarCloudT);
        pcl::transformPointCloud(*pDownsampled1,*pTransformed1,this->lidarInfoVec.at(0).extrinsics_eigen);
        pcl::transformPointCloud(*pDownsampled2,*pTransformed2,this->lidarInfoVec.at(1).extrinsics_eigen);

        LidarCloudT::Ptr pIntegrated(new LidarCloudT);
        *pIntegrated = (*pTransformed1) + (*pTransformed2);
        LidarCloudT::Ptr pGroundRemoved(new LidarCloudT);

        {
            ScopeTimer t("[lidar_points_integration] Ground removal timer:");
            removeGround(pIntegrated,pGroundRemoved,0.8);
//            pGroundRemoved = pIntegrated;
        }
        MessageCloudT pc;
        pcl::toROSMsg(*pGroundRemoved,pc);
//        MessageCloudT pc;
//        pcl::toROSMsg(*pIntegrated,pc);
        pc.header.frame_id = "lidar";
        pc.header.stamp = cloud_msg1->header.stamp;
        merged_cloud_pub.publish(pc);
    }
public:
    vector<LidarInfo> lidarInfoVec;

    void loadConfigFile()
    {
        cv::FileStorage fs;
        std::string lidar_config_file_path;
        if(! ros::param::get("lidar_config_file_path",lidar_config_file_path))
        {
            LOG(ERROR)<<"[lidar_points_integration] config file not set!"<<endl;
            throw "ERROR!";
        }
        fs.open(lidar_config_file_path,cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            LOG(ERROR)<<"[lidar_points_integration] no config file exist!"<<endl;
            throw "ERROR!";
        }
        cv::FileNode lidars_node = fs["Lidars"];
        if(lidars_node.type()!=cv::FileNode::SEQ)
        {
            LOG(ERROR)<<"[lidar_points_integration] format error!"<<endl;
        }
        cv::FileNodeIterator it = lidars_node.begin(), it_end = lidars_node.end(); // Go through the node
        for (; it != it_end; ++it)
        {
            LidarInfo info;
            (*it)["T_FLUBaselink_Lidar"]>>info.extrinsic_cv;
            (*it)["topic"]>>info.topic_name;
            (*it)["type"]>>info.lidar_type;
            (*it)["name"]>>info.name;
            cv::cv2eigen(info.extrinsic_cv,info.extrinsics_eigen);
            //info.cv2eigen();
            this->lidarInfoVec.push_back(info);
        }
    }
    void initNode(int argc,char** argv)
    {
        ros::init(argc,argv,"px4_state_reporter_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        loadConfigFile();
        if(!(ros::param::get("velodyne_downsampling_size",velodyne_downsampling_size)&&
             ros::param::get("livox_downsampling_size",livox_downsampling_size)))
        {
            LOG(ERROR)<<"Param not set in lidar_points_integration_node, quit!"<<endl;
            throw "Error!";
        }
        //merged_cloud_pub = pNH->advertise<MessageCloudT>("/gaas/preprocessing/merged_cloud",1);
        merged_cloud_pub = pNH->advertise<MessageCloudT>("/gaas/preprocessing/velodyne_downsampled",1);

        message_filters::Subscriber<MessageCloudT> lidar_sub0(*pNH, this->lidarInfoVec.at(0).topic_name, 1);
        message_filters::Subscriber<MessageCloudT> lidar_sub1(*pNH, this->lidarInfoVec.at(1).topic_name, 1);

        typedef message_filters::sync_policies::ApproximateTime<MessageCloudT, MessageCloudT> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar_sub0, lidar_sub1);
        sync.registerCallback(boost::bind(&LidarPointsIntegrationManager::sync2VelodyneCallback,this, _1, _2));
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
