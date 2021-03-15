#ifndef LIDAR_LOCALIZATION_NDT_ALGO_H
#define LIDAR_LOCALIZATION_NDT_ALGO_H

#include "typedefs.h"
#include <glog/logging.h>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include "../ndt_cpu/ndt_cpu_include/NormalDistributionsTransform.h"
#include "Timer.h"



class NDTAlgo
{
public:
    MapCloudT::Ptr pmap_cloud=nullptr;

    //temp!
    bool ever_init = false;
    typedef cpu::NormalDistributionsTransform<MapPointT,LidarPointT> NDT_CORE;
    typedef Eigen::Matrix<float, 4, 4> RTMatrix4f;
    pcl::NormalDistributionsTransform<MapPointT, LidarPointT>::Matrix4 prev_res; //sb template....


    bool loadPCDMap()
    {
        string map_path;
        bool path_exist = ros::param::get("map_path",map_path);
        if(!path_exist)
        {
            LOG(ERROR)<<"Fatal error in lidar_localization_ndt: get map_path failed!"<<endl;
            LOG(ERROR)<<"map_path:"<<map_path<<endl;
            exit(-1);
        }
        pmap_cloud = MapCloudT::Ptr(new MapCloudT);
        pcl::io::loadPCDFile(map_path, *pmap_cloud);
        if(pmap_cloud->size()>0)
        {
            LOG(INFO)<<"map pointcloud size:"<<pmap_cloud->size()<<endl;
            return true;
        }
        return false;
    }
    //    bool do_ndt_matching_without_initial_guess(LidarCloudT::Ptr pcloud_current)
    //    {
    //        pcl::NormalDistributionsTransform<MapPointT, LidarPointT> ndt;
    //        ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
    //        ndt.setStepSize (0.1);
    //        ndt.setResolution (2.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    //        ndt.setMaximumIterations (10);  //Setting max number of registration iterations.


    //        ndt.setInputSource (pcloud_current);// Setting point cloud to be aligned.
    //        ndt.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
    //        LidarCloudT::Ptr output_cloud (new LidarCloudT);
    //        ndt.align(*output_cloud);
    //        if(ndt.hasConverged()&&ndt.getEuclideanFitnessEpsilon()<0.5)
    //        {
    //            LOG(INFO)<<"NDT Converged. Transformation:"<<ndt.getFinalTransformation()<<endl;
    //            LOG(INFO)<<"Iteration times:"<<ndt.getFinalNumIteration()<<endl;
    //            return true;
    //        }
    //        LOG(INFO)<<"NDT matching failed. Epsilon:"<<ndt.getEuclideanFitnessEpsilon()<<endl;
    //        return false;
    //    }





    bool do_ndt_matching_without_initial_guess2(LidarCloudT::Ptr pcloud_current,RTMatrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
    {
        //pcl::NormalDistributionsTransform<MapPointT, LidarPointT> ndt;
        ScopeTimer ndt_timer("ndt_timer");
        NDT_CORE ndt;

        //ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.

        ndt.setStepSize (0.1);
        ndt.setResolution (1.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).


        ndt.setInputTarget(pmap_cloud);

        ndt.setInputSource (pcloud_current);// Setting point cloud to be aligned.
        ndt_timer.watch("till input set.");
        //ndt.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
        LidarCloudT::Ptr output_cloud (new LidarCloudT);
        if(!this->ever_init)
        {
            ndt.setMaximumIterations (30);  //Setting max number of registration iterations.
            ndt.setTransformationEpsilon (0.05); // Setting maximum step size for More-Thuente line search.
            RTMatrix4f initialguess;
            initialguess<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
            ndt.align(*output_cloud,initialguess);
        }
        else
        {//set prev result as initial guess.
            ndt.setMaximumIterations(30);
            ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
            LOG(INFO)<<"ndt with initial guess"<<endl;
            ndt.align(*output_cloud,this->prev_res);
        }
        ndt_timer.watch("till ndt aligned.");
        if(ndt.hasConverged())//&&ndt.getEuclideanFitnessEpsilon()<0.5)
        {
            this->ever_init = true;
            output_pose = ndt.getFinalTransformation();
            this->prev_res = ndt.getFinalTransformation();
            LOG(INFO)<<"NDT Converged. Transformation:"<<ndt.getFinalTransformation()<<endl;
            LOG(INFO)<<"Iteration times:"<<ndt.getFinalNumIteration()<<endl;
            if(need_transformed_cloud)
            {
                LidarCloudT::Ptr transformed(new LidarCloudT);
                pcl::transformPointCloud(*pcloud_current,*transformed,output_pose);
                transformed_cloud_ptr = transformed;
            }
            return true;
        }
        //LOG(INFO)<<"NDT matching failed. Epsilon:"<<ndt.getEuclideanFitnessEpsilon()<<endl;
        LOG(INFO)<<"NDT matching failed."<<endl;
        return false;
    }
    //    bool do_ndt_matching_with_initial_guess(LidarCloudT::Ptr pcloud_current,Pose3d Pose)
    //    {
    //        ;
    //    }

    //private:
    //    bool do_ndt_matching()
    //    {
    //        return false;
    //    }
    //    double evalutateNDTResult()
    //    {
    //        return 0.0;
    //    }
};



#endif
