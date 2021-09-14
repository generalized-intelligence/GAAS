#ifndef NDT_LOCALIZATION_ALGORITHM_H
#define NDT_LOCALIZATION_ALGORITHM_H

#include "localization_algorithm_abstract.h"

class NDTLocalizationAlgorithm:public LocalizationAlgorithmAbstract
{
private:
    bool doNDTMatching();
public:
    bool initLocalizationAlgorithm(ros::NodeHandle& nh);
    bool doMatchingWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                        Eigen::Matrix4f& pose_guess,
                                        Eigen::Matrix4f& output_pose,
                                        string initial_guess_type="gps_ahrs"//"gps_ahrs","icp_prev","imu_preint"
                );
    bool doMatching(LidarCloudT::Ptr pcloud_current,Eigen::Matrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,
                                   bool need_transformed_cloud,const ros::Time& cloud_stamp);
};

#endif
