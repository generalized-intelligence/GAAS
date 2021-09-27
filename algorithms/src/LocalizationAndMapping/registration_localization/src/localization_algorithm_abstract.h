#ifndef LOCALIZATION_ALGORITHM_ABSTRACT
#define LOCALIZATION_ALGORITHM_ABSTRACT

#include <ros/ros.h>
#include "registration_map_manager.h"
#include "GPS_AHRS_sync.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/TransformStamped.h"

class LocalizationAlgorithmAbstract
{
protected:
    GPS_AHRS_Synchronizer::Ptr pGPS_AHRS_Sync;
    bool poseGuessEverInit = false;

    Eigen::Matrix4f gps_ahrs_initial_guess;
    bool gps_ahrs_initial_avail = false;

    Eigen::Matrix4f last_result;
    bool last_result_avail = false;

    bool localization_ever_init = false;//进行过成功定位

    MapCloudT::Ptr mapBuffer = nullptr;

    double downsample_size = -1;

    //imu preint
    bool enable_imu_preint = false;
    tf2_ros::Buffer tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> pTFListener;

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

    virtual bool init(ros::NodeHandle& nh,MapCloudT::Ptr buffer,GPS_AHRS_Synchronizer::Ptr pGPS_AHRS_Sync_in,double downsample_size_in)
    {
        //for imu preint
        if(!ros::param::get("enable_imu_preint_initial_guess",enable_imu_preint))
        {
            LOG(ERROR)<<"[registration localizaiton] Param not set in LocalizationAlgorithm::init()."<<endl;
            throw "error!";
        }
        //init imu preint
        if(enable_imu_preint)
        {
            pTFListener = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(this->tf2_buffer));
            LOG(INFO)<<"[registration_localization] Enable IMU Preint."<<endl;
        }
        this->loadMapBuffer(buffer);
        this->pGPS_AHRS_Sync = pGPS_AHRS_Sync_in;
        this->downsample_size = downsample_size_in;
        this->initLocalizationAlgorithm(nh);
        LOG(INFO)<<"Localization algorithm initialized!"<<endl;
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
        LOG(INFO)<<"[registration_localization] in lidar_callback() with algorithm:"<<this->getMatchingAlgorithmType()<<endl;
        //step<1> get init pose estimation.
        bool last_result_is_avail = queryLastResultAvail();
        Eigen::Matrix4f initial_pose,output_pose;

        LidarCloudT::Ptr pInputCloud(new LidarCloudT);
        pcl::fromROSMsg(*pLidarMsg, *pInputCloud);

        if(!last_result_is_avail)
        {
            getInitialPoseWithGPSAndAHRS(initial_pose,map_manager_binding);
        }
        else
        {
            getInitialPoseGuess(initial_pose,map_manager_binding);
        }
        auto currentMapCloud = map_manager_binding->getCurrentMapCloud(initial_pose);

        bool localization_result_valid = false;
        Eigen::Matrix4f imu_preint_pose_initial_guess;
        if(enable_imu_preint&&get_imu_preint_odometry(pLidarMsg->header.stamp,imu_preint_pose_initial_guess))
        {
            localization_result_valid = doMatchingWithInitialPoseGuess(pInputCloud,map_manager_binding->getCurrentMapCloud(initial_pose),imu_preint_pose_initial_guess,output_pose,output_cloud,"imu_preint");
            int counter = 2;
            while(!localization_result_valid&&counter)
            {//retry once since we are using imu_preint result...
                LOG(WARNING)<<"[ICP Matching] Retry once, do icp for 5 more times due to using inaccurate imu_preint initial guess."<<endl;
                Eigen::Matrix4f guess = output_pose;
                localization_result_valid = doMatchingWithInitialPoseGuess(pInputCloud,map_manager_binding->getCurrentMapCloud(initial_pose),guess,output_pose,output_cloud,"imu_preint");
                counter--;
            }
        }
        else
        {
            if(this->localization_ever_init)
            {
                localization_result_valid = doMatchingWithInitialPoseGuess(pInputCloud,map_manager_binding->getCurrentMapCloud(initial_pose),initial_pose,output_pose,output_cloud,"prev_result");
            }
            else
            {
                localization_result_valid = doMatchingWithInitialPoseGuess(pInputCloud,map_manager_binding->getCurrentMapCloud(initial_pose),initial_pose,output_pose,output_cloud,"gps_ahrs");
            }
        }
        if(localization_result_valid)
        {
            last_result_avail = true;
            localization_ever_init = true;
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
bool LocalizationAlgorithmAbstract::get_imu_preint_odometry(const ros::Time &cloud_stamp, Eigen::Matrix4f &output_pose)
{
    if(!this->localization_ever_init)
    {
        return false;
    }
    geometry_msgs::TransformStamped tf_stamped;
    try
    {
        std::string canTransform_error_str;
        if(!tf2_buffer.canTransform("map","imu_preint",cloud_stamp,ros::Duration(0.001),&canTransform_error_str))
        {
            LOG(WARNING)<<"[registration localization] in canTransform(): not avail! Reason:"<<canTransform_error_str<<endl;
            return false;
        }
        tf_stamped = this->tf2_buffer.lookupTransform("map","imu_preint",cloud_stamp,ros::Duration(0.001));//1ms timeout. Ensure the procedure is realtime.
    }
    catch (const std::exception e)
    {
        LOG(WARNING)<<"[registration localization] Exception caught in get_imu_preint_odometry() while looking up imu_preint transformation."<<endl;
        return false;
    }
    if( fabs((tf_stamped.header.stamp - cloud_stamp).toSec())>0.06 )
    {
        LOG(WARNING)<<"[registration localization] imu_preint and lidar cloud latency>60ms, imu_preint rejected."<<endl;
        return false;
    }
    {
        Eigen::Matrix4f prev_res_copy = this->last_result;
        const auto& R = tf_stamped.transform.rotation;
        const auto& t = tf_stamped.transform.translation;
        Eigen::Quaternionf q(R.w,R.x,R.y,R.z);
        output_pose.block(0,0,3,3) = q.toRotationMatrix();
        output_pose(0,3) = t.x;
        output_pose(1,3) = t.y;
        output_pose(2,3) = t.z;//TODO: see if rotation-only setting will work?
        double position_error_square = pow( t.x- prev_res_copy(0,3) ,2) + pow( t.y - prev_res_copy(1,3) ,2) + pow( t.z - prev_res_copy(2,3) ,2);
        if( position_error_square > 4.5*4.5)//4.5m in 0.5s  --> 162 km/h
        {
            LOG(WARNING)<<"[registration localization] Estimated velocity too large, ignore imu preint result.Position error:"<<sqrt(position_error_square)<<endl;
            return false;
        }
        Eigen::Matrix3f prev_rot = prev_res_copy.block(0,0,3,3);
        Eigen::Quaternionf f2(prev_rot);
        const float ROTATION_THRES = 45*3.1415926/180; // 45 deg.
        if(f2.angularDistance(q)>ROTATION_THRES) // check rotation.
        {
            LOG(WARNING)<<"[registration localization] Estimated angular rate too large, ignore imu preint result.Rot error:"<<f2.angularDistance(q)*180.0/3.14<<"deg."<<endl;
            return false;
        }
        LOG(INFO)<<"[registration localization] Using imu_preint result which was checked."<<endl;


        //Using rotation only.
        LOG(INFO)<<"[registration localization] Using imu predicted rotation only."<<endl;
        output_pose(0,3) = prev_res_copy(0,3);
        output_pose(1,3) = prev_res_copy(1,3);
        return true;
    }

}
#endif
