#ifndef ICP_LOCALIZATION_ALGORITHM_H
#define ICP_LOCALIZATION_ALGORITHM_H

#include "localization_algorithm_abstract.h"
#include <pcl/registration/gicp.h>
#include "fast_gicp/gicp/fast_vgicp.hpp"
#include "fast_gicp/gicp/fast_vgicp_cuda.hpp"


class ICPLocalizationAlgorithm:public LocalizationAlgorithmAbstract
{
protected:
    typedef fast_gicp::FastVGICPCuda<MapPointT,LidarPointT> ICP_CORE;
    ICP_CORE icp;
    //ICP Params.
    double icp_transformation_epsilon,icp_resolution,icp_max_correspondence_distance,icp_euclidean_fitness_epsilon;
    bool initLocalizationAlgorithm(ros::NodeHandle& nh);
    void loadMapBuffer(MapCloudT::Ptr buffer)  // 加载点云buffer 有些方法可能需要缓存加速
    {
        ScopeTimer load_map_buffer_timer("load_map_buffer_timer");
        this->mapBuffer = buffer;
        icp.setInputTarget(this->mapBuffer);
    }

public:
    std::string getMatchingAlgorithmType()
    {
        return std::string("ICP");
    }
    bool doMatchingWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                        MapCloudT::Ptr pmap_current,
                                        Eigen::Matrix4f& pose_guess,
                                        Eigen::Matrix4f& output_pose,
                                        LidarCloudT::Ptr& output_cloud_,
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
    else
    {
        icp.setTransformationEpsilon(icp_transformation_epsilon);
        icp.setResolution(icp_resolution);
        icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
        icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
        LOG(INFO)<<"ICP initialized with icp_transformation_epsilon="<<icp_transformation_epsilon<<";icp_resolution="<<icp_resolution
                <<";icp_max_correspondence_distance="<<icp_max_correspondence_distance
                <<";icp_euclidean_fitness_epsilon="<<icp_euclidean_fitness_epsilon<<"."<<endl;
    }
}
bool ICPLocalizationAlgorithm::doMatchingWithInitialPoseGuess(LidarCloudT::Ptr pcloud_current,
                                                              MapCloudT::Ptr pmap_current,
                                                              Eigen::Matrix4f &pose_guess,
                                                              Eigen::Matrix4f &output_pose,
                                                              LidarCloudT::Ptr& output_cloud_,
                                                              std::string initial_guess_type)
{
    ScopeTimer icp_timer("icp_timer");
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



    //Downsampling
    LidarCloudT::Ptr cloud_downsampled(new LidarCloudT);
    pcl::VoxelGrid<LidarPointT> sor;
    sor.setInputCloud(pcloud_current);
    sor.setLeafSize(downsample_size, downsample_size, downsample_size);
    sor.filter(*cloud_downsampled);
    pcloud_current = cloud_downsampled;



    //icp.setInputTarget(pmap_cloud);
    icp.setInputSource (pcloud_current);// Setting point cloud to be aligned.
    icp_timer.watch("till input set.");
    //icp.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
    LidarCloudT::Ptr output_cloud (new LidarCloudT);
    output_cloud_ = output_cloud;
    if(initial_guess_type == "gps_ahrs")
    {
        //icp.setResolution (3.0);  //Setting Resolution of icp grid structure (VoxelGridCovariance).
        //icp.setStepSize (0.5);
        icp.setMaximumIterations (20);  //Setting max number of registration iterations.
        //icp.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
        LOG(INFO)<<"icp with prev icp initial guess"<<endl;
        icp.align(*output_cloud,pose_guess);
    }
    else if(initial_guess_type == "prev_result")
    {
        //icp.setResolution (2.0);  //Setting Resolution of icp grid structure (VoxelGridCovariance).
        //icp.setStepSize (0.5);
        icp.setMaximumIterations (10);  //Setting max number of registration iterations.
        //icp.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
        LOG(INFO)<<"icp with prev initial guess"<<endl;
        icp.align(*output_cloud,pose_guess);
    }
    else if(initial_guess_type == "imu_preint")
    {
        icp.setMaximumIterations (5);  //Setting max number of registration iterations.
        LOG(INFO)<<"[ICP Matching] icp with imu preint initial guess"<<endl;
        icp.align(*output_cloud,pose_guess);
    }


    if(!icp.hasConverged()&&icp.getFitnessScore()>10.0)
    {
        LOG(WARNING)<<"ICP with "<<initial_guess_type<<" initial guess failed! Fitness score:"<<icp.getFitnessScore()<<endl;
        return false;
    }
    output_pose = icp.getFinalTransformation();
    LOG(INFO)<<"ICP Converged. Transformation:"<<icp.getFinalTransformation()<<endl;
    LOG(INFO)<<"ICP with initial guess "<<initial_guess_type<<" finished."<<endl;
    return true;
}

#endif
