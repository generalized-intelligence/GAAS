#ifndef NDT_LOCALIZATION_ALGORITHM_H
#define NDT_LOCALIZATION_ALGORITHM_H

#include "localization_algorithm_abstract.h"
#include <glog/logging.h>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#ifdef CUDA_FOUND  //To use ndt cpu version.
    #undef CUDA_FOUND
#endif

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
    std::string getMatchingAlgorithmType()
    {
#ifdef CUDA_FOUND
    return std::string("NDT GPU");
#else
    return std::string("NDT CPU");
#endif
    }
    bool initLocalizationAlgorithm(ros::NodeHandle& nh);
    void loadMapBuffer(MapCloudT::Ptr buffer)  // 加载点云buffer 有些方法可能需要缓存加速
    {
        ScopeTimer load_map_buffer_timer("load_map_buffer_timer");
        this->mapBuffer = buffer;
        ndt.setInputTarget(mapBuffer);
    }
public:

    bool doMatchingWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                         MapCloudT::Ptr pmap_current,
                                         Eigen::Matrix4f& pose_guess,
                                         Eigen::Matrix4f& output_pose,
                                         LidarCloudT::Ptr& output_cloud_,
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
                                                              LidarCloudT::Ptr& output_cloud_,
                                                              string initial_guess_type//"gps_ahrs","result_prev","imu_preint"
        )
{
    ScopeTimer ndt_timer("ndt_timer");
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
    auto original_size = pcloud_current->size();

    //Downsampling
    LidarCloudT::Ptr cloud_downsampled(new LidarCloudT);
    pcl::VoxelGrid<LidarPointT> sor;
    sor.setInputCloud(pcloud_current);
    sor.setLeafSize(downsample_size, downsample_size, downsample_size);
    sor.filter(*cloud_downsampled);
    pcloud_current = cloud_downsampled;

    LOG(INFO)<<"Map size:"<<pmap_current->size()<<"; lidar cloud size:"<<pcloud_current->size()<<";before downsampling size:"<<original_size<<endl;


    //ndt.setInputTarget(pmap_cloud);
    ndt.setInputSource (pcloud_current);// Setting point cloud to be aligned.
    ndt_timer.watch("till input set.");
    //ndt.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
    LidarCloudT::Ptr output_cloud (new LidarCloudT);
    output_cloud_ = output_cloud;
    if(initial_guess_type == "gps_ahrs")
    {
        //ndt.setResolution (3.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        //ndt.setStepSize (0.5);
        ndt.setMaximumIterations (10);  //Setting max number of registration iterations.
        ndt.setTransformationEpsilon (0.05); // Setting maximum step size for More-Thuente line search.
        LOG(INFO)<<"ndt with prev ndt initial guess"<<endl;
#ifdef CUDA_FOUND
        ndt.align(pose_guess);
#else
        ndt.align(*output_cloud,pose_guess);
#endif
    }
    else if(initial_guess_type == "prev_result")
    {
        //ndt.setResolution (2.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        //ndt.setStepSize (0.5);
        ndt.setMaximumIterations (10);  //Setting max number of registration iterations.
        ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
        LOG(INFO)<<"ndt with prev ndt initial guess"<<endl;
#ifdef CUDA_FOUND
        ndt.align(pose_guess);
#else
        ndt.align(*output_cloud,pose_guess);
#endif
    }

    else if(initial_guess_type == "imu_preint")
    {
        ndt.setMaximumIterations (5);  //Setting max number of registration iterations.
        ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
        LOG(INFO)<<"ndt with prev ndt initial guess"<<endl;
#ifdef CUDA_FOUND
        ndt.align(pose_guess);
#else
        ndt.align(*output_cloud,pose_guess);
#endif
    }



    if(!ndt.hasConverged())
    {
        LOG(WARNING)<<"NDT with "<<initial_guess_type<<" initial guess failed!"<<endl;
        return false;
    }
    output_pose = ndt.getFinalTransformation();
    LOG(INFO)<<"NDT Converged. Transformation:"<<ndt.getFinalTransformation()<<endl;
    LOG(INFO)<<"NDT with initial guess "<<initial_guess_type<<" finished; Iteration times:"<<ndt.getFinalNumIteration()<<endl;    
    return true;
}
#endif
