#include "MapBlock.h"
//将perception模块的点云逐个转换成障碍地图坐标系 发布占据网格信息.

#include <iostream>
#include <mutex>

#include <pcl/filters/conditional_removal.h>

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

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "gaas_msgs/GAASPerceptionObstacleCluster.h"
#include "gaas_msgs/GAASPerceptionObstacleClustersList.h"

#include "gaas_msgs/GAASNavigationDynamicBlockGrid.h"
#include "gaas_msgs/GAASNavigationDynamicBlockMap.h"
#include <pcl/registration/ndt.h>

#include "Timer.h"


class DynamicBlockGenerator
{
public:
    MapCloudT::Ptr pmap_cloud;
    MapBlock map_block;
    MapBlock dynamic_block_map_empty;
    std::shared_ptr<ros::NodeHandle> pNH;
    ros::Subscriber perception_subscriber;
    ros::Publisher dynamic_map_publisher;
    std::shared_ptr<tf2_ros::TransformListener> pTFListener;
    tf2_ros::Buffer tfBuffer;
    double DYNAMIC_BLOCK_GEN_DOWNSAMPLING_SIZE = 0.1;
    double PERCEPTION_OBSTACLE_RANGE = 30;
    void initDynamicBlockGenerator(int argc,char** argv)
    {
        //init ros
        ros::init(argc,argv,"dynamic_block_generator_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        perception_subscriber = pNH->subscribe<gaas_msgs::GAASPerceptionObstacleClustersList>
                ("/gaas/perception/euclidean_original_clusters_list",
                 1,&DynamicBlockGenerator::obstacleClustersListCallback,this);
        dynamic_map_publisher = pNH->advertise<gaas_msgs::GAASNavigationDynamicBlockMap>
                ("/gaas/navigation/dynamic_block_map",1);
        pTFListener = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tfBuffer));
        //init original HD map.

        if( (!ros::param::get("dynamic_block_generator_downsampling_size",DYNAMIC_BLOCK_GEN_DOWNSAMPLING_SIZE))||
                (!ros::param::get("dynamic_block_obstacle_range",PERCEPTION_OBSTACLE_RANGE))
                )
        {
            LOG(ERROR)<<"Error: no dynamic_block_generator_downsampling_size set in launch file!"<<endl;
            throw "Error!";
        }
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
        //set empty dynamic map.
        map_block.initMapBlock(pmap_cloud);
        dynamic_block_map_empty = map_block;
        dynamic_block_map_empty.eraseBasicBlocks();
        //spin
        ros::spin();
    }


    void obstacleClustersListCallback(const gaas_msgs::GAASPerceptionObstacleClustersListConstPtr& clusters)
    {
        ScopeTimer t_("in obstacleClustersListCallback():");
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("lidar", "map",
                                                        clusters->header.stamp,ros::Duration(0.01));//等待10ms 超时. 如果有imu预积分信息，那么这个频率会很高，一定能取到.
        }
        catch (tf2::TransformException &ex)
        {
            LOG(WARNING)<<"query lidar->map tf timeout!Using old tf info.Info:"<<ex.what()<<endl;
            try
            {
                transformStamped = tfBuffer.lookupTransform("lidar", "map",ros::Time(0));//直接取最后一个
            }
            catch(tf2::TransformException &ex2)
            {
                LOG(ERROR)<<"ERROR: No lidar->map tf in buffer!Obstacle clusters transformation failed. Info:"<<ex2.what()<<endl;
                return;
            }
        }
        t_.watch("till tf transform get:");
        Eigen::Matrix4f transformMat;
        Eigen::Quaternionf quat;
        quat.x() = transformStamped.transform.rotation.x;
        quat.y() = transformStamped.transform.rotation.y;
        quat.z() = transformStamped.transform.rotation.z;
        quat.w() = transformStamped.transform.rotation.w;

        Eigen::Matrix3f rot = quat.toRotationMatrix();
        transformMat.block(0,0,3,3) = rot;
        transformMat(0,3) = transformStamped.transform.translation.x;
        transformMat(1,3) = transformStamped.transform.translation.y;
        transformMat(2,3) = transformStamped.transform.translation.z;

        LidarCloudT::Ptr total_perception_clusters_map_coord(new LidarCloudT);

        pcl::ConditionAnd<LidarPointT>::Ptr range_cond (new pcl::ConditionAnd<LidarPointT>());
        range_cond->addComparison (pcl::FieldComparison<LidarPointT>::ConstPtr (new
                                                                                pcl::FieldComparison<LidarPointT> ("x", pcl::ComparisonOps::GT, -PERCEPTION_OBSTACLE_RANGE)));
        range_cond->addComparison (pcl::FieldComparison<LidarPointT>::ConstPtr (new
                                                                                pcl::FieldComparison<LidarPointT> ("x", pcl::ComparisonOps::LT, PERCEPTION_OBSTACLE_RANGE)));
        range_cond->addComparison (pcl::FieldComparison<LidarPointT>::ConstPtr (new
                                                                                pcl::FieldComparison<LidarPointT> ("y", pcl::ComparisonOps::GT, -PERCEPTION_OBSTACLE_RANGE)));
        range_cond->addComparison (pcl::FieldComparison<LidarPointT>::ConstPtr (new
                                                                                pcl::FieldComparison<LidarPointT> ("y", pcl::ComparisonOps::LT, PERCEPTION_OBSTACLE_RANGE)));
        range_cond->addComparison (pcl::FieldComparison<LidarPointT>::ConstPtr (new
                                                                                pcl::FieldComparison<LidarPointT> ("z", pcl::ComparisonOps::GT, -PERCEPTION_OBSTACLE_RANGE)));
        range_cond->addComparison (pcl::FieldComparison<LidarPointT>::ConstPtr (new
                                                                                pcl::FieldComparison<LidarPointT> ("z", pcl::ComparisonOps::LT, PERCEPTION_OBSTACLE_RANGE)));
        // build the filter
        pcl::ConditionalRemoval<LidarPointT> condrem;
        condrem.setCondition (range_cond);

        condrem.setKeepOrganized(false);
        // apply filter


        for(auto& obs:clusters->obstacles)
        {
            //to pcl pointcloud
            LidarCloudT::Ptr pObsPCL(new LidarCloudT);
            pcl::fromROSMsg(obs.cloud,*pObsPCL);
            //range limit
            LidarCloudT::Ptr pRangeLimited(new LidarCloudT);
            condrem.setInputCloud (pObsPCL);
            condrem.filter (*pRangeLimited);
            //downsampling
            LidarCloudT::Ptr downsampled_input(new LidarCloudT);
            pcl::VoxelGrid<LidarPointT> sor;
            sor.setInputCloud(pRangeLimited);
            sor.setLeafSize(DYNAMIC_BLOCK_GEN_DOWNSAMPLING_SIZE, DYNAMIC_BLOCK_GEN_DOWNSAMPLING_SIZE, DYNAMIC_BLOCK_GEN_DOWNSAMPLING_SIZE);
            sor.filter(*downsampled_input);
            LidarCloudT::Ptr transformed(new LidarCloudT);
            //transform to map coordinate.
            pcl::transformPointCloud(*downsampled_input,*transformed,transformMat);

            //do update dynamic block.
            //            total_perception_clusters_map_coord->points.insert
            //                    (total_perception_clusters_map_coord->points.begin(),
            //                    );
            std::copy(transformed->points.begin(),transformed->points.end(),std::back_inserter(total_perception_clusters_map_coord->points));
        }
        t_.watch("till whole cloud got:");
        //update current map.
        MapBlock dynamic_block = this->dynamic_block_map_empty;
        dynamic_block.setBlockOccupancy(*total_perception_clusters_map_coord);//设置障碍本身占用.
        t_.watch("till Occupancy set:");
        //dynamic_block.doConvolutionExpansion();//扩张 TODO:这个函数没实现.
        //publish dynamic blocks.
        std::shared_ptr<gaas_msgs::GAASNavigationDynamicBlockMap> p_dynamic_block_map_msg = dynamic_block.toROSMsg(clusters->header);
        t_.watch("till msg generated:");
        dynamic_map_publisher.publish(*p_dynamic_block_map_msg);
        t_.watch("till msg published:");
    }

};


int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("dynamic_block_generator_node");
    DynamicBlockGenerator blockGen;
    blockGen.initDynamicBlockGenerator(argc,argv);
    return 0;
}
