
#include "registration_map_manager.h"
#include "ndt_localization_algorithm.h"
#include "icp_localization_algorithm.h"
#include "GPS_AHRS_sync.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"

class RegistrationLocalizationNode
{
private:
    const bool need_transformed_pointcloud = false;

    std::shared_ptr<LocalizationAlgorithmAbstract> pLocalizationAlgorithm;
    RegistrationMapManager::Ptr pMapManager;
    GPS_AHRS_Synchronizer::Ptr gps_ahrs_sync;


    ros::Subscriber lidarSubscriber;
    ros::Publisher pointCloudPublisher,registrationPosePublisher,GPS_AHRS_PosePublisher;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > pSync;

    std::shared_ptr< message_filters::Subscriber<sensor_msgs::NavSatFix> >pGPSsub;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > pAHRSsub;

    float lidar_height_to_gps = 0.0;//TODO: implement this compensation.


    bool initMapManager(ros::NodeHandle& nh);
public:
    void initRegistrationNode(ros::NodeHandle& nh)
    {
        initMapManager(nh);
        gps_ahrs_sync = GPS_AHRS_Synchronizer::Ptr(new GPS_AHRS_Synchronizer);

        bool use_ndt = false;
        double downsample_size = -1;
        if(! (ros::param::get("use_ndt",use_ndt) && ros::param::get("downsample_size",downsample_size)
              &&ros::param::get("lidar_height_to_gps",lidar_height_to_gps)
              ))
        {
            LOG(ERROR)<<"[registration localization] registration method not set in launch file!"<<endl;
        }

        //init localization algorithm.
        if(use_ndt)
        {
            std::shared_ptr<NDTLocalizationAlgorithm> pNDT(new NDTLocalizationAlgorithm);
            pLocalizationAlgorithm = std::static_pointer_cast<LocalizationAlgorithmAbstract>(pNDT);
        }
        else
        {
            std::shared_ptr<ICPLocalizationAlgorithm> pICP(new ICPLocalizationAlgorithm);
            pLocalizationAlgorithm = std::static_pointer_cast<LocalizationAlgorithmAbstract>(pICP);
        }
        pLocalizationAlgorithm->init(nh,this->pMapManager->getCurrentMapCloudBuffer(),gps_ahrs_sync,downsample_size);
        //bindCallbacks(nh);
        {//bind callbacks.
            registrationPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/gaas/localization/registration_pose",1);
            pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/gaas/visualization/localization/registration_merged_cloud",1);
            GPS_AHRS_PosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/gaas/localization/original_gps_ahrs_pose",1);
            pGPSsub = std::shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix> >(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "/mavros/global_position/raw/fix", 1) );
            pAHRSsub = std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> >(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/mavros/global_position/local", 1) );


            // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
            pSync = std::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *pGPSsub, *pAHRSsub) );
            pSync->registerCallback(boost::bind(&RegistrationLocalizationNode::gps_ahrs_callback,this, _1, _2));

            LOG(INFO)<<"[registration_localization] waiting for gps-ahrs msgs..."<<endl;
            //wait until gps-ahrs synchronizer initialized;
            while(!gps_ahrs_sync->ever_init)
            {
                ros::spinOnce();
            }
            lidarSubscriber = nh.subscribe("/gaas/preprocessing/velodyne_downsampled",1,&RegistrationLocalizationNode::lidar_callback,this);
            LOG(INFO)<<"[registration_localization] bindCallbacks finished!"<<endl;
        }
        ros::spin();
    }
    // Services callback

    void bindCallbacks(ros::NodeHandle& nh);

    bool loadMapServiceCallback();//加载地图 内部函数
    bool unloadMapServiceCallback();//卸载地图 内部函数
    bool reloadNewMapServiceCallback(); //换装新地图 内部函数
    bool getCurrentMapServiceCallback(); //查看当前地图名

    // Subscribers callback
    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& pLidarMsg)
    {
        ScopeTimer timer("lidar_callback timer");
        Eigen::Matrix4f output_pose;
        LidarCloudT::Ptr transformed_cloud;
        if(this->pLocalizationAlgorithm->lidar_callback(pLidarMsg,pMapManager,output_pose,transformed_cloud))
        {

            //publish output pose;
            Eigen::Matrix3f rotmat = output_pose.block(0,0,3,3);
            Eigen::Quaternionf quat(rotmat);
            LOG(INFO)<<"quat.norm()="<<quat.norm()<<endl;

            geometry_msgs::PoseStamped registration_pose_msg;

            registration_pose_msg.header.frame_id="map";//地图坐标系的定位.
            registration_pose_msg.header.stamp = pLidarMsg->header.stamp;//与雷达同步.

            auto& pos = registration_pose_msg.pose.position;
            auto& orient = registration_pose_msg.pose.orientation;
            orient.x = quat.x();
            orient.y = quat.y();
            orient.z = quat.z();
            orient.w = quat.w();

            pos.x = output_pose(0,3);
            pos.y = output_pose(1,3);
            pos.z = output_pose(2,3);

            timer.watch("before publishing pose:");
            registrationPosePublisher.publish(registration_pose_msg);

            timer.watch("till pose published:");

//            //publish tf2 transfrom.
//            Eigen::Matrix4f lidar_to_map = output_pose.inverse();
//            Eigen::Matrix3f l_m_rotmat = lidar_to_map.block(0,0,3,3);
//            Eigen::Quaternionf l_m_quat(l_m_rotmat);
//            LOG(INFO)<<"l_m_quat.norm()="<<l_m_quat.norm()<<endl;



//            geometry_msgs::TransformStamped map_lidar_trans_stamped;
//            map_lidar_trans_stamped.header.frame_id = "lidar";// we've got lidar as the top node of tf tree.
//            //So when localization is not avail, still we can solve lidar to body.
//            map_lidar_trans_stamped.child_frame_id = "map";

//            map_lidar_trans_stamped.header.stamp = pLidarMsg->header.stamp;
//            map_lidar_trans_stamped.transform.translation.x = lidar_to_map(0,3);
//            map_lidar_trans_stamped.transform.translation.y = lidar_to_map(1,3);
//            map_lidar_trans_stamped.transform.translation.z = lidar_to_map(2,3);
//            map_lidar_trans_stamped.transform.rotation.x = l_m_quat.x();
//            map_lidar_trans_stamped.transform.rotation.y = l_m_quat.y();
//            map_lidar_trans_stamped.transform.rotation.z = l_m_quat.z();
//            map_lidar_trans_stamped.transform.rotation.w = l_m_quat.w();
            //publish tf2 transfrom.
            Eigen::Matrix4f lidar_to_map = output_pose;
            Eigen::Matrix3f l_m_rotmat = lidar_to_map.block(0,0,3,3);
            Eigen::Quaternionf l_m_quat(l_m_rotmat);
            LOG(INFO)<<"l_m_quat.norm()="<<l_m_quat.norm()<<endl;



            geometry_msgs::TransformStamped map_lidar_trans_stamped;
            map_lidar_trans_stamped.header.frame_id = "map";// we've got lidar as the top node of tf tree.
            //So when localization is not avail, still we can solve lidar to body.
            map_lidar_trans_stamped.child_frame_id = "lidar";

            map_lidar_trans_stamped.header.stamp = pLidarMsg->header.stamp;
            map_lidar_trans_stamped.transform.translation.x = lidar_to_map(0,3);
            map_lidar_trans_stamped.transform.translation.y = lidar_to_map(1,3);
            map_lidar_trans_stamped.transform.translation.z = lidar_to_map(2,3);
            map_lidar_trans_stamped.transform.rotation.x = l_m_quat.x();
            map_lidar_trans_stamped.transform.rotation.y = l_m_quat.y();
            map_lidar_trans_stamped.transform.rotation.z = l_m_quat.z();
            map_lidar_trans_stamped.transform.rotation.w = l_m_quat.w();


//end



            tf_broadcaster.sendTransform(map_lidar_trans_stamped);

            if(need_transformed_pointcloud&&transformed_cloud!=nullptr)
            {
                sensor_msgs::PointCloud2 transformed_cloud_msg;
                pcl::toROSMsg(*transformed_cloud,transformed_cloud_msg);
                transformed_cloud_msg.header.frame_id = "lidar";
                transformed_cloud_msg.header.stamp = pLidarMsg->header.stamp;
                pointCloudPublisher.publish(transformed_cloud_msg);
            }
            //publish merged pointcloud
        }
        else
        {
            LOG(ERROR)<<"Localization with"<<this->pLocalizationAlgorithm->getMatchingAlgorithmType()<<" failed!"<<endl;
        }
    }
    void gps_ahrs_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom);// 同步gps和ahrs.
};

bool RegistrationLocalizationNode::initMapManager(ros::NodeHandle& nh)
{
    pMapManager = RegistrationMapManager::Ptr(new RegistrationMapManager);
    pMapManager->init(nh);
}

void RegistrationLocalizationNode::gps_ahrs_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom)
{
    LOG(INFO)<<"In gps_ahrs_callback"<<endl;
    gps_ahrs_sync->sync_mutex.lock();
    gps_ahrs_sync->gps_msg = *gps_msg;
    gps_ahrs_sync->ahrs_msg = *odom;
    gps_ahrs_sync->ever_init = true;
    gps_ahrs_sync->sync_mutex.unlock();

    {
        Eigen::Matrix4f gps_ahrs_initial_guess;
        this->pLocalizationAlgorithm->getInitialPoseWithGPSAndAHRS(gps_ahrs_initial_guess,pMapManager);
        Eigen::Matrix3f rotmat = gps_ahrs_initial_guess.block(0,0,3,3);
        Eigen::Quaternionf quat(rotmat);
        geometry_msgs::PoseStamped gps_ahrs_pose_msg;

        gps_ahrs_pose_msg.header.frame_id="map";//地图坐标系的定位.
        gps_ahrs_pose_msg.header.stamp = gps_msg->header.stamp;//与gps同步.

        auto& pos = gps_ahrs_pose_msg.pose.position;
        auto& orient = gps_ahrs_pose_msg.pose.orientation;
        orient.x = quat.x();
        orient.y = quat.y();
        orient.z = quat.z();
        orient.w = quat.w();

        pos.x = gps_ahrs_initial_guess(0,3);
        pos.y = gps_ahrs_initial_guess(1,3);
        pos.z = gps_ahrs_initial_guess(2,3);

        GPS_AHRS_PosePublisher.publish(gps_ahrs_pose_msg);
        LOG(INFO)<<"GPS AHRS pose published!"<<endl;
    }

    this->pLocalizationAlgorithm->gps_ahrs_callback();
};
//void RegistrationLocalizationNode::bindCallbacks(ros::NodeHandle& nh)
//{
//    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/mavros/global_position/raw/fix", 1);
//    message_filters::Subscriber<nav_msgs::Odometry> ahrs_sub(nh, "/mavros/global_position/local", 1);

//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry> MySyncPolicy;
//    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, ahrs_sub);
//    sync.registerCallback(boost::bind(&RegistrationLocalizationNode::gps_ahrs_callback,this, _1, _2));

//    LOG(INFO)<<"[registration_localization] waiting for gps-ahrs msgs..."<<endl;
//    //wait until gps-ahrs synchronizer initialized;
//    while(!gps_ahrs_sync->ever_init)
//    {
//        ros::spinOnce();
//    }
//    lidarSubscriber = nh.subscribe("/gaas/preprocess/velodyne_donwsampled",1,&RegistrationLocalizationNode::lidar_callback,this);
//    LOG(INFO)<<"[registration_localization] bindCallbacks() finished!"<<endl;
//}
int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("registration_localization_node");
    ros::init(argc,argv,"registration_localization_node");
    std::shared_ptr<ros::NodeHandle> pNH(new ros::NodeHandle);
    RegistrationLocalizationNode node;
    node.initRegistrationNode(*pNH);
    return 0;
}
