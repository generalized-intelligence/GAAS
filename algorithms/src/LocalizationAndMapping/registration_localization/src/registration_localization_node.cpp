#include "registraion_map_manager.h"
#include "ndt_localization_algorithm.h"
#include "icp_localization_algorithm.h"
#include "GPS_AHRS_sync.h"

class RegistrationLocalizationNode
{
private:
    std::shared_ptr<LocalizationAlgorithmAbstract> pLocalizationAlgorithm;
    RegistrationMapManager::Ptr pMapManager;
    GPS_AHRS_Synchronizer::Ptr gps_ahrs_sync;
    const bool use_icp = true;
    bool initMapManager(ros::NodeHandle& nh);
    void startLoop();
public:
    void initRegistrationNode(ros::NodeHandle& nh)
    {
        initMapManager(nh);
        gps_ahrs_sync = GPS_AHRS_Synchronizer::Ptr(new GPS_AHRS_Synchronizer);
        if(use_icp)
        {
            std::shared_ptr<ICPLocalizationAlgorithm> pICP(new ICPLocalizationAlgorithm);
            pLocalizationAlgorithm = std::static_pointer_cast<LocalizationAlgorithmAbstract>(pICP);
        }
        pLocalizationAlgorithm->initLocalizationModule(nh,pMapManager,gps_ahrs_sync);
        bindCallbacks();
        startLoop();
    }
    // Services callback

    void bindCallbacks();

    bool loadMapServiceCallback();//加载地图 内部函数
    bool unloadMapServiceCallback();//卸载地图 内部函数
    bool reloadNewMapServiceCallback(); //换装新地图 内部函数
    bool getCurrentMapServiceCallback(); //查看当前地图名

    // Subscribers callback
    void lidar_callback()
    {

        //step<1> get init pose estimation.
        bool ever_init_pose_guess = pLocalizationAlgorithm->queryPoseGuessEverInit();
        if(!ever_init_pose_guess)
        {
            pLocalizationAlgorithm->getInitialPoseWithGPSAndAHRS();
        }
        auto initial_pose = pLocalizationAlgorithm->getInitialPoseGuess();

        auto currentMapCloud = pMapManager->getCurrentMapCloud(initial_pose);
        Eigen::Matrix4f output_pose;
        bool localization_result_valid = pLocalizationAlgorithm->doMatchingWithInitialPoseGuess(currentMapCloud,initial_pose,output_pose);


    }
    void gps_ahrs_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom);// 同步gps和ahrs.
};

void RegistrationLocalizationNode::gps_ahrs_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom)
{
    LOG(INFO)<<"In gps_ahrs_callback"<<endl;
    gps_ahrs_sync->sync_mutex.lock();
    gps_ahrs_sync->gps_msg = *gps_msg;
    gps_ahrs_sync->ahrs_msg = *odom;
    gps_ahrs_sync->ever_init = true;
    gps_ahrs_sync->sync_mutex.unlock();
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"registration_localization_node");
    std::shared_ptr<ros::NodeHandle> pNH(new ros::NodeHandle);
    RegistrationLocalizationNode node;
    node.initRegistrationNode(*pNH);
    return 0;
}
