#ifndef LIDAR_LOCALIZATION_NDT_ALGO_H
#define LIDAR_LOCALIZATION_NDT_ALGO_H

#include "typedefs.h"
#include <glog/logging.h>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include "fast_gicp/gicp/fast_vgicp.hpp"
#include "fast_gicp/gicp/fast_vgicp_cuda.hpp"


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



class ICPAlgo
{
public:
    MapCloudT::Ptr pmap_cloud=nullptr;
    MapGPSInfo map_gps_info;

    //temp!
    bool ever_init = false;

    typedef fast_gicp::FastVGICPCuda<MapPointT,LidarPointT> ICP_CORE;

    //typedef Eigen::Matrix<float, 4, 4> RTMatrix4f;
    Eigen::Matrix4f prev_res;



    Eigen::Matrix4f gps_ahrs_initial_guess;
    bool gps_ahrs_initial_avail = false;
    GPS_AHRS_Synchronizer* p_gps_ahrs_sync;

    bool lidar_height_compensation = false;
    double lidar_height_to_gps=0;
    ICP_CORE icp;


    ICPAlgo(GPS_AHRS_Synchronizer* gps_ahrs_sync_ptr)
    {
        this->p_gps_ahrs_sync = gps_ahrs_sync_ptr;
        if(ros::param::get("lidar_height_to_gps",lidar_height_to_gps)&&ros::param::get("icp_downsample_size",DOWNSAMPLE_SIZE))
        {
            LOG(INFO)<<"LIDAR_GPS_HEIGHT_COMPENSATION ready!"<<endl;
            lidar_height_compensation = true;
        }
        else
        {
            LOG(ERROR)<<"init icp localizer failed!"<<endl;
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
            LOG(ERROR)<<"Fatal error in icp_lidar_localization: get map_path failed!"<<endl;
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



        double resolution_in = 10.0;
        double tranformation_epsilon_in = 0.1;
        double max_correspondence_distance_in = 5.0;
        double euclidean_fitness_epsilon_in = 0.2;

        if(ros::param::get("icp_transformation_epsilon",tranformation_epsilon_in)&&
                ros::param::get("icp_resolution",resolution_in)&&
                ros::param::get("icp_max_correspondence_distance",max_correspondence_distance_in)&&
                ros::param::get("icp_euclidean_fitness_epsilon",euclidean_fitness_epsilon_in)
                )
        {
            LOG(INFO)<<"ICP resolution:"<<resolution_in<<";transformation_epsilon:"<<tranformation_epsilon_in<<
                       ";max_correspondence_dist:"<<max_correspondence_distance_in<<";euclidean_fitness_epsilon:"<<euclidean_fitness_epsilon_in<<endl;
            //LOG(INFO)<<"ICP transformation_epsilon:"<<tranformation_epsilon_in<<endl;
        }
        else
        {
            LOG(ERROR)<<"Can not get step size and resolution in launch file!"<<endl;
            throw "Error!";
        }

        icp.setResolution (resolution_in);  //Setting Resolution of ICP grid structure (VoxelGridCovariance).
        icp.setTransformationEpsilon(tranformation_epsilon_in);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_in); // 对应点最大距离，再大认为不对应。初值不准时放大。
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_in);//总欧氏距离误差。


//        icp.setCorrespondenceRandomness();
//        icp.setEuclideanFitnessEpsilon();
//        icp.setKernelWidth();

//        icp.setMaximumIterations();
//        icp.setNearestNeighborSearchMethod();
//        icp.setNeighborSearchMethod();
//        icp.setRANSACIterations();
//        icp.setRANSACOutlierRejectionThreshold();

        LOG(INFO)<<"[ICP_Matching] finished setting params."<<endl;


        icp.setInputTarget(pmap_cloud);
        LOG(INFO)<<"finished setInputTarget()"<<endl;
        LOG(INFO)<<"in ICPAlgo: map loaded!"<<endl;
        return flag_map;
    }

    bool initializeICPPoseGuess()
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


//    //bool do_ndt_matching_without_initial_guess2(LidarCloudT::Ptr pcloud_current,RTMatrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
//    bool DEBUG_visualize_gps_ahrs_initialguess(LidarCloudT::Ptr pcloud_current,RTMatrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
//    {
//        //pcl::NormalDistributionsTransform<MapPointT, LidarPointT> ndt;
//        ScopeTimer ndt_timer("ndt_timer");
//        NDT_CORE ndt;

//        //ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.

//        ndt.setStepSize (0.1);
//        ndt.setResolution (1.0);  //Setting Resolution of NDT grid structure (VoxelGridCovariance).


//        ndt.setInputTarget(pmap_cloud);

//        ndt.setInputSource (pcloud_current);// Setting point cloud to be aligned.
//        ndt_timer.watch("till input set.");
//        //ndt.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
//        LidarCloudT::Ptr output_cloud (new LidarCloudT);
//        if(!this->ever_init)
//        {
//            ndt.setMaximumIterations (10);  //Setting max number of registration iterations.
//            ndt.setTransformationEpsilon (0.05); // Setting maximum step size for More-Thuente line search.
//            RTMatrix4f initialguess;
//            if(gps_ahrs_initial_avail)
//            {
//                initialguess = gps_ahrs_initial_guess;
//                LOG(INFO)<<"Using gps ahrs intitial guess";
//            }
//            else
//            {
//                return false;
//                initialguess<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
//            }
//#ifdef CUDA_FOUND
//            ndt.align(initialguess);
//#else
//            ndt.align(*output_cloud,initialguess);
//#endif

//        }

//        else
//        {//set prev result as initial guess.
//            ndt.setMaximumIterations(10);
//            ndt.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
//            LOG(INFO)<<"ndt with initial guess"<<endl;
//            //ndt.align(*output_cloud,this->prev_res);
//#ifdef CUDA_FOUND
//            ndt.align(gps_ahrs_initial_guess);
//#else
//            ndt.align(*output_cloud,gps_ahrs_initial_guess);
//#endif
//        }


//        ndt_timer.watch("till ndt aligned.");
//        if(ndt.hasConverged())//&&ndt.getEuclideanFitnessEpsilon()<0.5)
//        {
//            this->ever_init = true;
//            output_pose = ndt.getFinalTransformation();
//            this->prev_res = ndt.getFinalTransformation();
//            LOG(INFO)<<"NDT Converged. Transformation:"<<ndt.getFinalTransformation()<<endl;
//            LOG(INFO)<<"Iteration times:"<<ndt.getFinalNumIteration()<<endl;
//            if(need_transformed_cloud)
//            {
//                LidarCloudT::Ptr transformed(new LidarCloudT);
//                //pcl::transformPointCloud(*pcloud_current,*transformed,output_pose);
//                pcl::transformPointCloud(*pcloud_current,*transformed,gps_ahrs_initial_guess);
//                transformed_cloud_ptr = transformed;
//            }
//            return true;
//        }
//        //LOG(INFO)<<"NDT matching failed. Epsilon:"<<ndt.getEuclideanFitnessEpsilon()<<endl;
//        LOG(INFO)<<"NDT matching failed."<<endl;
//        return false;
//    }

    bool doICPWithInitialPoseGuess (LidarCloudT::Ptr pcloud_current,
                                    Eigen::Matrix4f& pose_guess,
                                    Eigen::Matrix4f& output_pose,
                                    string initial_guess_type="gps_ahrs"//"gps_ahrs","icp_prev","imu_integrated"
            )
    {
        ScopeTimer icp_timer("icp_timer");



        //icp.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
        LidarCloudT::Ptr cloud_downsampled(new LidarCloudT);
        pcl::VoxelGrid<LidarPointT> sor;
        sor.setInputCloud(pcloud_current);
        sor.setLeafSize(DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE, DOWNSAMPLE_SIZE);
        sor.filter(*cloud_downsampled);
        pcloud_current = cloud_downsampled;



        //icp.setInputTarget(pmap_cloud);
        icp.setInputSource (pcloud_current);// Setting point cloud to be aligned.
        icp_timer.watch("till input set.");
        //icp.setInputTarget (pmap_cloud);// Setting point cloud to be aligned to.
        LidarCloudT::Ptr output_cloud (new LidarCloudT);
        if(initial_guess_type == "gps_ahrs")
        {
            //icp.setResolution (3.0);  //Setting Resolution of ICP grid structure (VoxelGridCovariance).
            //icp.setStepSize (0.5);
            icp.setMaximumIterations (10);  //Setting max number of registration iterations.
            //icp.setTransformationEpsilon (0.01); // Setting maximum step size  for More-Thuente line search. Set by config file.
            LOG(INFO)<<"icp with prev icp initial guess"<<endl;
            icp.align(*output_cloud,pose_guess);
        }
        else if(initial_guess_type == "icp_prev")
        {
            //icp.setResolution (2.0);  //Setting Resolution of ICP grid structure (VoxelGridCovariance).
            //icp.setStepSize (0.5);
            //icp.setMaximumIterations (10);  //Setting max number of registration iterations.
            //icp.setTransformationEpsilon (0.01); // Setting maximum step size for More-Thuente line search.
            LOG(INFO)<<"icp with prev icp initial guess"<<endl;
            //icp.align(pose_guess);
            icp.align(*output_cloud,pose_guess);

        }

        if(!icp.hasConverged()&&icp.getFitnessScore()>5.0)
        {
            LOG(WARNING)<<"ICP with "<<initial_guess_type<<" initial guess failed!Fitness score:"<<icp.getFitnessScore()<<endl;
            return false;
        }
        output_pose = icp.getFinalTransformation();
        LOG(INFO)<<"ICP Converged. Transformation:"<<icp.getFinalTransformation()<<endl;
        LOG(INFO)<<"ICP with initial guess "<<initial_guess_type<<" finished." <<endl;
        return true;
    }

    bool doICPMatching(LidarCloudT::Ptr pcloud_current,Eigen::Matrix4f& output_pose,LidarCloudT::Ptr& transformed_cloud_ptr,bool need_transformed_cloud = false)
    {
        bool flag_icp_success = false;
        if(ever_init)
        {//使用上次icp初始值进行初始化.
            flag_icp_success = doICPWithInitialPoseGuess(pcloud_current,this->prev_res,output_pose,"icp_prev");
        }
        else
        {//尝试使用GPS_AHRS初始化.
            if(gps_ahrs_initial_avail)//
            {
                flag_icp_success = doICPWithInitialPoseGuess(pcloud_current,this->gps_ahrs_initial_guess,output_pose,"gps_ahrs");
            }
            else
            {//没有初始值,放弃.
                return false;
            }
        }
        if(flag_icp_success)
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
};



#endif
