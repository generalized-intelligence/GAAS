#ifndef LOCALIZATION_ALGORITHM_ABSTRACT
#define LOCALIZATION_ALGORITHM_ABSTRACT

#include "registraion_map_manager.h"
#include "GPS_AHRS_sync.h"

class LocalizationAlgorithmAbstract
{
private:
    GPS_AHRS_Synchronizer::Ptr pGPS_AHRS_Sync;
    RegistrationMapManager::Ptr map_manager_binding;
    bool poseGuessEverInit = false;


public:
    bool initLocalizationModule(ros::NodeHandle& nh,RegistrationMapManager::Ptr map_manager,GPS_AHRS_Synchronizer::Ptr sync);
    bool queryPoseGuessEverInit()
    {
        return this->poseGuessEverInit;
    }
    bool initPoseGuess();
    Eigen::Matrix4f getInitialPoseGuess();
    virtual bool initLocalizationAlgorithm(ros::NodeHandle& nh) = 0;

    bool get_imu_preint_odometry(const ros::Time& cloud_stamp, Eigen::Matrix4f& output_pose);

    virtual bool doMatchingWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                        Eigen::Matrix4f& pose_guess,
                                        Eigen::Matrix4f& output_pose,
                                        string initial_guess_type="gps_ahrs"//"gps_ahrs","icp_prev","imu_preint"
                )=0;
    virtual bool doMatching(LidarCloudT::Ptr pcloud_current,Eigen::Matrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,
                                   bool need_transformed_cloud,const ros::Time& cloud_stamp) = 0;


    void getInitialPoseWithGPSAndAHRS();
// Callbacks
    std::string getMatchingAlgorithmType();
};


//bool LocalizationAlgorithmAbstract::loadPCDMap()
//{
//    //通过pcd文件配置动态加载地图.
//}
bool LocalizationAlgorithmAbstract::initLocalizationModule(ros::NodeHandle& nh,RegistrationMapManager::Ptr map_manager,GPS_AHRS_Synchronizer::Ptr sync)
{
    this->map_manager_binding = map_manager; //create map_manager binding.
    initLocalizationAlgorithm(nh);
    initPoseGuess();
}

#endif
