
#include "registraion_map_manager.h"
#include "ndt_localization_algorithm.h"
#include "icp_localization_algorithm.h"
#include "GPS_AHRS_sync.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
class RegistrationLocalizationNode
{
private:
    std::shared_ptr<LocalizationAlgorithmAbstract> pLocalizationAlgorithm;
    RegistrationMapManager::Ptr pMapManager;
    GPS_AHRS_Synchronizer::Ptr gps_ahrs_sync;
    const bool use_ndt = true;


    ros::Subscriber lidarSubscriber;

    bool initMapManager(ros::NodeHandle& nh);
public:
    void initRegistrationNode(ros::NodeHandle& nh)
    {
        initMapManager(nh);
        gps_ahrs_sync = GPS_AHRS_Synchronizer::Ptr(new GPS_AHRS_Synchronizer);

        //init localization algorithm.
        if(use_ndt)
        {
            std::shared_ptr<NDTLocalizationAlgorithm> pNDT(new NDTLocalizationAlgorithm);
            pLocalizationAlgorithm = std::static_pointer_cast<LocalizationAlgorithmAbstract>(pNDT);
        }
        pLocalizationAlgorithm->init(nh,this->pMapManager->getCurrentMapCloudBuffer());
        bindCallbacks(nh);
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
        this->pLocalizationAlgorithm->lidar_callback(pLidarMsg,pMapManager);
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
    this->pLocalizationAlgorithm->gps_ahrs_callback();
};
void RegistrationLocalizationNode::bindCallbacks(ros::NodeHandle& nh)
{
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "image1", 1);
    message_filters::Subscriber<nav_msgs::Odometry> ahrs_sub(nh, "image2", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, ahrs_sub);
    sync.registerCallback(boost::bind(&RegistrationLocalizationNode::gps_ahrs_callback,this, _1, _2));

    //wait until gps-ahrs synchronizer initialized;
    while(!gps_ahrs_sync->ever_init)
    {
        ros::spinOnce();
    }

    lidarSubscriber = nh.subscribe("/gaas/preprocess/velodyne_donwsampled",1,&RegistrationLocalizationNode::lidar_callback,this);
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"registration_localization_node");
    std::shared_ptr<ros::NodeHandle> pNH(new ros::NodeHandle);
    RegistrationLocalizationNode node;
    node.initRegistrationNode(*pNH);
    return 0;
}
