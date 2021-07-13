#ifndef LIDAR_LOCALIZATION_NDT_ALGO_H
#define LIDAR_LOCALIZATION_NDT_ALGO_H

#include "typedefs.h"
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

#include "Timer.h"
#include "GPS_AHRS_sync.h"
#include <opencv2/core/persistence.hpp>

struct MapGPSInfo
{
    double longitude;
    double latitude;
    double altitude;
    string coordinate_mode;

    const double earth_radius_m = 6371393;
    const double pi_ = 3.1415926535;
    void getRelativeXYZFromLonLatAltInNWUCoordinate(double lon,double lat,double alt,double& x,double& y,double& z)
    {
        x = ((lat-latitude)*pi_/180.0)*earth_radius_m;
        y = ((longitude-lon)*pi_/180.0)*cos(latitude*pi_/180)*earth_radius_m;
        z = alt-altitude;
    }
};
float DOWNSAMPLE_SIZE = 0.3;//0.2
//const float DOWNSAMPLE_SIZE = 50;//0.2



class NDTAlgo
{
public:
    MapCloudT::Ptr pmap_cloud=nullptr;
    MapGPSInfo map_gps_info;

    //temp!
    bool ever_init = false;
#ifdef CUDA_FOUND
    typedef gpu::GNormalDistributionsTransform NDT_CORE;
#else
    typedef cpu::NormalDistributionsTransform<MapPointT,LidarPointT> NDT_CORE;
#endif
    typedef Eigen::Matrix<float, 4, 4> RTMatrix4f;
    pcl::NormalDistributionsTransform<MapPointT, LidarPointT>::Matrix4 prev_res; //sb template....


    Eigen::Matrix4f gps_ahrs_initial_guess;
    bool gps_ahrs_initial_avail = false;
    GPS_AHRS_Synchronizer* p_gps_ahrs_sync;

    bool lidar_height_compensation = false;
    double lidar_height_to_gps=0;
    NDT_CORE ndt;


    NDTAlgo(GPS_AHRS_Synchronizer* gps_ahrs_sync_ptr)
    {
        this->p_gps_ahrs_sync = gps_ahrs_sync_ptr;
        if(ros::param::get("lidar_height_to_gps",lidar_height_to_gps)&&ros::param::get("ndt_downsample_size",DOWNSAMPLE_SIZE))
        {
            LOG(INFO)<<"LIDAR_GPS_HEIGHT_COMPENSATION ready!"<<endl;
            lidar_height_compensation = true;
        }
        else
        {
            LOG(ERROR)<<"init ndt localizer failed!"<<endl;
            throw "Error";
        }

    }

    bool loadPCDMap()
    {
        bool flag_map = false;
        bool flag_gps_config = false;
        string map_path;
        bool path_exist = ros::param::get("map_path",map_path);
        if(!path_exist)
        {
            LOG(ERROR)<<"Fatal error in lidar_localization_ndt: get map_path failed!"<<endl;
            LOG(ERROR)<<"map_path:"<<map_path<<endl;
            exit(-1);
        }
        MapCloudT::Ptr pFullMap(new MapCloudT);

        this->pmap_cloud = MapCloudT::Ptr(new MapCloudT);
        pcl::io::loadPCDFile(map_path, *pFullMap);
        if(pFullMap->size()<=0)
        {
            LOG(ERROR)<<"In loadPCDMap(): Map empty!"<<endl;
            throw "Error!";
        }

        pcl::VoxelGrid<LidarPointT> sor;
        sor.setInputCloud(pFullMap);
        sor.setLeafSize(DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE);
        sor.filter(*this->pmap_cloud);


        if(pmap_cloud->size()>0)
        {
            LOG(INFO)<<"map pointcloud size:"<<pmap_cloud->size()<<endl;
            flag_map = true;
        }
        cv::FileStorage map_gps_config;
        map_gps_config.open(map_path+".yaml",cv::FileStorage::READ);
        if(!map_gps_config.isOpened())
        {
            LOG(ERROR)<<"Map config file not found!"<<endl;
            throw "Error";
        }
        map_gps_config["initial_longitude"]>>this->map_gps_info.longitude;
        map_gps_config["initial_latitude"]>>this->map_gps_info.latitude;
        map_gps_config["initial_altitude"]>>this->map_gps_info.altitude;
        map_gps_config["coordinate_mode"]>>this->map_gps_info.coordinate_mode;
        if(this->map_gps_info.coordinate_mode!="NWU")
        {
            LOG(ERROR)<<"Coordinate mode is not NWU; not supported yet!"<<endl;
            throw "Error";
        }

        //ndt.setResolution (3.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).

        double step_size_in=2.0;
        double resolution_in = 10.0;

        if(ros::param::get("ndt_step_size",step_size_in)&&ros::param::get("ndt_resolution",resolution_in))
        {
            LOG(INFO)<<"NDT step size:"<<step_size_in<<"; resolution:"<<resolution_in<<endl;
        }
        else
        {
            LOG(ERROR)<<"Can not get step size and resolution in launch file!"<<endl;
            throw "Error!";
        }

        ndt.setResolution (step_size_in);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        LOG(INFO)<<"finished setresolution()"<<endl;
        //ndt.setStepSize (0.5);
        ndt.setStepSize (resolution_in);
        LOG(INFO)<<"finished setStepSize()"<<endl;

        ndt.setInputTarget(pmap_cloud);
        LOG(INFO)<<"finished setInputTarget()"<<endl;

        LOG(INFO)<<"in NDTAlgo: map loaded!"<<endl;
        return flag_map;
    }

    bool initializeNDTPoseGuess()
    {
        sensor_msgs::NavSatFix gps;
        nav_msgs::Odometry ahrs;
        if(this->p_gps_ahrs_sync->get_sync_gps_ahrs_msgs(gps,ahrs))
        {
            double x,y,z;
            this->map_gps_info.getRelativeXYZFromLonLatAltInNWUCoordinate(gps.longitude,gps.latitude,gps.altitude,x,y,z);
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
            gps_ahrs_initial_guess(2,3) = xyz_(2) - lidar_height_compensation;
            gps_ahrs_initial_guess(3,3) = 1;
            LOG(INFO)<<"GPS_AHRS_INITIAL:"<<endl<<gps_ahrs_initial_guess<<endl;

            gps_ahrs_initial_avail = true;
            return true;
        }
        else
        {
            LOG(INFO)<<"gps_ahrs not initialized."<<endl;
            return false;
        }
    }


    //bool do_ndt_matching_without_initial_guess2(LidarCloudT::Ptr pcloud_current,RTMatrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
    bool DEBUG_visualize_gps_ahrs_initialguess(LidarCloudT::Ptr pcloud_current,RTMatrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
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
            ndt.setMaximumIterations (10);  //Setting max number of registration iterations.
            ndt.setTransformationEpsilon (0.05); // Setting maximum step size for More-Thuente line search.
            RTMatrix4f initialguess;
            if(gps_ahrs_initial_avail)
            {
                initialguess = gps_ahrs_initial_guess;
                LOG(INFO)<<"Using gps ahrs intitial guess";
            }
            else
            {
                return false;
                initialguess<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
            }
#ifdef CUDA_FOUND
            ndt.align(initialguess);
#else
            ndt.align(*output_cloud,initialguess);
#endif

        }

        else
        {//set prev result as initial guess.
            ndt.setMaximumIterations(10);
            ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
            LOG(INFO)<<"ndt with initial guess"<<endl;
            //ndt.align(*output_cloud,this->prev_res);
#ifdef CUDA_FOUND
            ndt.align(gps_ahrs_initial_guess);
#else
            ndt.align(*output_cloud,gps_ahrs_initial_guess);
#endif
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
                //pcl::transformPointCloud(*pcloud_current,*transformed,output_pose);
                pcl::transformPointCloud(*pcloud_current,*transformed,gps_ahrs_initial_guess);
                transformed_cloud_ptr = transformed;
            }
            return true;
        }
        //LOG(INFO)<<"NDT matching failed. Epsilon:"<<ndt.getEuclideanFitnessEpsilon()<<endl;
        LOG(INFO)<<"NDT matching failed."<<endl;
        return false;
    }

    bool doNDTWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                    Eigen::Matrix4f& pose_guess,
                                    Eigen::Matrix4f& output_pose,
                                    string initial_guess_type="gps_ahrs"//"gps_ahrs","ndt_prev","imu_integrated"
            )
    {
        ScopeTimer ndt_timer("ndt_timer");



        //ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
        LidarCloudT::Ptr cloud_downsampled(new LidarCloudT);
        pcl::VoxelGrid<LidarPointT> sor;
        sor.setInputCloud(pcloud_current);
        sor.setLeafSize(DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE);
        sor.filter(*cloud_downsampled);
        pcloud_current = cloud_downsampled;



        //ndt.setInputTarget(pmap_cloud);
        ndt.setInputSource (pcloud_current);// Setting point cloud to be aligned.
        ndt_timer.watch("till input set.");
        //ndt.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
        LidarCloudT::Ptr output_cloud (new LidarCloudT);
        if(initial_guess_type == "gps_ahrs")
        {
            //ndt.setResolution (3.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
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
        else if(initial_guess_type == "ndt_prev")
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

    bool doNDTMatching(LidarCloudT::Ptr pcloud_current,Eigen::Matrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
    {
        bool flag_ndt_success = false;
        if(ever_init)
        {//使用上次ndt初始值进行初始化.
            flag_ndt_success = doNDTWithInitialPoseGuess(pcloud_current,this->prev_res,output_pose,"ndt_prev");
        }
        else
        {//尝试使用GPS_AHRS初始化.
            if(gps_ahrs_initial_avail)//
            {
                flag_ndt_success = doNDTWithInitialPoseGuess(pcloud_current,this->gps_ahrs_initial_guess,output_pose,"gps_ahrs");
            }
            else
            {//没有初始值,放弃.
                return false;
            }
        }
        if(flag_ndt_success)
        {
            // 点云坐标变换并发布
            this->ever_init = true;
            this->prev_res = output_pose;

            if(need_transformed_cloud)
            {
                LidarCloudT::Ptr transformed(new LidarCloudT);
                //pcl::transformPointCloud(*pcloud_current,*transformed,output_pose);
                pcl::transformPointCloud(*pcloud_current,*transformed,output_pose);
                transformed_cloud_ptr = transformed;
            }
            return true;
        }
    }

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
