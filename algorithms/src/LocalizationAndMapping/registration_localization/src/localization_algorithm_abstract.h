#ifndef LOCALIZATION_ALGORITHM_ABSTRACT
#define LOCALIZATION_ALGORITHM_ABSTRACT

#include <ros/ros.h>
#include "registraion_map_manager.h"
#include "GPS_AHRS_sync.h"

class LocalizationAlgorithmAbstract
{
protected:
    GPS_AHRS_Synchronizer::Ptr pGPS_AHRS_Sync;
    bool poseGuessEverInit = false;

    Eigen::Matrix4f gps_ahrs_initial_guess;
    bool gps_ahrs_initial_avail = false;

    Eigen::Matrix4f last_result;
    bool last_result_avail = false;

    MapCloudT::Ptr mapBuffer = nullptr;

    virtual bool initLocalizationAlgorithm(ros::NodeHandle& nh) = 0;
    virtual void loadMapBuffer(MapCloudT::Ptr buffer)  // 加载点云buffer 有些方法可能需要缓存加速
    {
        this->mapBuffer = buffer;
    }


public:
    bool queryPoseGuessEverInit()
    {
        return this->poseGuessEverInit;
    }
    bool queryLastResultAvail()
    {
        return last_result_avail;
    }
    bool getInitialPoseWithGPSAndAHRS(Eigen::Matrix4f& output_pose,RegistrationMapManager::Ptr map_manager_binding);
    bool getInitialPoseGuess(Eigen::Matrix4f& output_pose,RegistrationMapManager::Ptr map_manager_binding);

    virtual bool init(ros::NodeHandle& nh,MapCloudT::Ptr buffer,GPS_AHRS_Synchronizer::Ptr pGPS_AHRS_Sync_in)
    {
        this->loadMapBuffer(buffer);
        this->pGPS_AHRS_Sync = pGPS_AHRS_Sync_in;
        this->initLocalizationAlgorithm(nh);
    }

    bool get_imu_preint_odometry(const ros::Time& cloud_stamp, Eigen::Matrix4f& output_pose);

    virtual bool doMatchingWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                        MapCloudT::Ptr pmap_current,
                                        Eigen::Matrix4f& pose_guess,
                                        Eigen::Matrix4f& output_pose,
                                        LidarCloudT::Ptr& output_cloud_,
                                        string initial_guess_type="gps_ahrs"//"gps_ahrs","prev_result","imu_preint"
                )=0;

// Callbacks
    virtual std::string getMatchingAlgorithmType() = 0;

    bool lidar_callback(const sensor_msgs::PointCloud2ConstPtr& pLidarMsg,RegistrationMapManager::Ptr map_manager_binding,Eigen::Matrix4f& output_pose_,LidarCloudT::Ptr& output_cloud)
    {
        LOG(INFO)<<"[registration_localization] in lidar_callback():"<<endl;
        //step<1> get init pose estimation.
        bool last_result_is_avail = queryLastResultAvail();
        Eigen::Matrix4f initial_pose,output_pose;
        if(!last_result_is_avail)
        {
            getInitialPoseWithGPSAndAHRS(initial_pose,map_manager_binding);
        }
        else
        {
            getInitialPoseGuess(initial_pose,map_manager_binding);
        }
        auto currentMapCloud = map_manager_binding->getCurrentMapCloud(initial_pose);
        //TODO: add visualization.

        bool localization_result_valid = doMatchingWithInitialPoseGuess(currentMapCloud,map_manager_binding->getCurrentMapCloud(initial_pose),initial_pose,output_pose,output_cloud);
        if(localization_result_valid)
        {
            last_result_avail = true;
            output_pose_ = output_pose;
            this->last_result = output_pose;
        }
        else
        {
            last_result_is_avail = false;
        }
        return localization_result_valid;
    }
    void gps_ahrs_callback()
    {
        this->poseGuessEverInit = true;
    }
    virtual ~LocalizationAlgorithmAbstract()
    {
        ;
    }
};
bool LocalizationAlgorithmAbstract::getInitialPoseGuess(Eigen::Matrix4f &output_pose, RegistrationMapManager::Ptr map_manager_binding)
{
    output_pose = this->last_result;
    return true;//TODO: do we need mutex?
}

bool LocalizationAlgorithmAbstract::getInitialPoseWithGPSAndAHRS(Eigen::Matrix4f& output_pose,RegistrationMapManager::Ptr map_manager_binding)
{
    sensor_msgs::NavSatFix gps;
    nav_msgs::Odometry ahrs;
    if(this->pGPS_AHRS_Sync->get_sync_gps_ahrs_msgs(gps,ahrs))
    {
        double x,y,z;
        map_manager_binding->getCurrentMap()->map_gps_info.getRelativeXYZFromLonLatAltInNWUCoordinate(gps.longitude,gps.latitude,gps.altitude,x,y,z);
        LOG(INFO)<<"GPS relative XYZ:"<<x<<";"<<y<<";"<<z<<endl;
        auto orient = ahrs.pose.pose.orientation;
        Eigen::Quaternionf original;
        original.x() = orient.x;
        original.y() = orient.y;
        original.z() = orient.z;
        original.w() = orient.w;


        //P_nwu = T_nwu_enu*T_enu_body*P_body
        Eigen::Matrix3f R_nwu_enu;
        R_nwu_enu<<0,1,0, -1,0,0 ,0,0,1; //plane right.
        Eigen::Matrix3f NWU_R = (R_nwu_enu*original.toRotationMatrix());

        Eigen::Quaternionf NWU_orient(NWU_R);
        LOG(INFO)<<"NWU coord quaternion initial guess = "<<NWU_orient.x()<<","<<NWU_orient.y()<<","<<NWU_orient.z()<<","<<NWU_orient.w()<<endl;
        gps_ahrs_initial_guess.block(0,0,3,3) = NWU_R;
        Eigen::Vector3f xyz_(x,y,z);

        gps_ahrs_initial_guess(0,3) = xyz_(0);
        gps_ahrs_initial_guess(1,3) = xyz_(1);
        gps_ahrs_initial_guess(2,3) = xyz_(2);//TODO: lidar height compensation.
        gps_ahrs_initial_guess(3,3) = 1;
        LOG(INFO)<<"GPS_AHRS_INITIAL:"<<endl<<gps_ahrs_initial_guess<<endl;
        output_pose = gps_ahrs_initial_guess;
        gps_ahrs_initial_avail = true;
        return  true;
    }
    else
    {
        LOG(INFO)<<"gps_ahrs not initialized."<<endl;
        return false;
    }
}
#endif
