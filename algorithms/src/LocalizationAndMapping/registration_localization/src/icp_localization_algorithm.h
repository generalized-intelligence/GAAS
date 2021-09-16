#ifndef ICP_LOCALIZATION_ALGORITHM_H
#define ICP_LOCALIZATION_ALGORITHM_H

#include "localization_algorithm_abstract.h"


class ICPLocalizationAlgorithm:public LocalizationAlgorithmAbstract
{
protected:
    //ICP Params.
    double icp_transformation_epsilon,icp_resolution,icp_max_correspondence_distance,icp_euclidean_fitness_epsilon;
    bool initLocalizationAlgorithm(ros::NodeHandle& nh);
    void loadMapBuffer(MapCloudT::Ptr buffer)  // 加载点云buffer 有些方法可能需要缓存加速
    {
        this->mapBuffer = buffer;
    }

public:
    bool doMatchingWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                        MapCloudT::Ptr pmap_current,
                                        Eigen::Matrix4f& pose_guess,
                                        Eigen::Matrix4f& output_pose,
                                        string initial_guess_type="gps_ahrs"//"gps_ahrs","prev_result","imu_preint"
                );

};

bool ICPLocalizationAlgorithm::initLocalizationAlgorithm(ros::NodeHandle &nh)
{
    if(!( ros::param::get("icp_transformation_epsilon",icp_transformation_epsilon)
          && ros::param::get("icp_resolution",icp_resolution)
          && ros::param::get("icp_max_correspondence_distance",icp_max_correspondence_distance)
          && ros::param::get("icp_euclidean_fitness_epsilon",icp_euclidean_fitness_epsilon) ))
    {
        LOG(ERROR)<<"In ICPLocalizationAlgorithm::initLocalizationAlgorithm(): param not set."<<endl;
        throw "Error!";
    }
}
bool ICPLocalizationAlgorithm::doMatchingWithInitialPoseGuess(LidarCloudT::Ptr pcloud_current,
                                                              MapCloudT::Ptr pmap_current,
                                                              Eigen::Matrix4f &pose_guess,
                                                              Eigen::Matrix4f &output_pose,
                                                              std::string initial_guess_type)
{
    throw "not implemented!";
    return true;
}

#endif
