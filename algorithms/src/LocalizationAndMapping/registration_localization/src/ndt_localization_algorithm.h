#ifndef NDT_LOCALIZATION_ALGORITHM_H
#define NDT_LOCALIZATION_ALGORITHM_H

#include "localization_algorithm_abstract.h"
#include <glog/logging.h>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#ifdef CUDA_FOUND
//    #define EIGEN_DONT_VECTORIZE
//    #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
    #include "../ndt_gpu/ndt_gpu_include/ndt_gpu/NormalDistributionsTransform.h"
#else
    #include "../ndt_cpu/ndt_cpu_include/NormalDistributionsTransform.h"
#endif

class NDTLocalizationAlgorithm:public LocalizationAlgorithmAbstract
{
protected:
#ifdef CUDA_FOUND
    typedef gpu::GNormalDistributionsTransform NDT_CORE;
#else
    typedef cpu::NormalDistributionsTransform<MapPointT,LidarPointT> NDT_CORE;
#endif

    NDT_CORE ndt;
    double ndt_step_size,ndt_resolution;
    bool doNDTMatching();
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

bool NDTLocalizationAlgorithm::initLocalizationAlgorithm(ros::NodeHandle &nh)
{
    if(!( ros::param::get("ndt_step_size",ndt_step_size)
          && ros::param::get("ndt_resolution",ndt_resolution)
          ))
    {
        LOG(ERROR)<<"In NDTLocalizationAlgorithm::initLocalizationAlgorithm(): param not set."<<endl;
        throw "Error!";
    }
    else
    {
        ndt.setResolution(ndt_resolution);
        ndt.setStepSize(ndt_step_size);
        LOG(INFO)<<"NDT initialized with resolution="<<ndt_resolution<<" and stepsize="<<ndt_step_size<<" ."<<endl;
    }
    return true;
}
bool NDTLocalizationAlgorithm::doMatchingWithInitialPoseGuess(LidarCloudT::Ptr pcloud_current,
                                                              MapCloudT::Ptr pmap_current,
                                                              Eigen::Matrix4f& pose_guess,
                                                              Eigen::Matrix4f& output_pose,
                                                              string initial_guess_type//"gps_ahrs","result_prev","imu_preint"
        )
{
    //检查buffer.
    if(pmap_current!=this->mapBuffer)
    {//load new buffer
        if(pmap_current==nullptr)
        {
            LOG(ERROR)<<"[doMatchingWithInitialPoseGuess] Map is empty!"<<endl;
            return false;
        }
        loadMapBuffer(pmap_current);
    }
    bool match_success = false;



    return match_success;


}
#endif
