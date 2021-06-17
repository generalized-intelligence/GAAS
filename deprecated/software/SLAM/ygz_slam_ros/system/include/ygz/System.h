#ifndef YGZ_SYSTEM_H
#define YGZ_SYSTEM_H

#include "ygz/NumTypes.h"
#include "ygz/Tracker.h"
#include "ygz/BackendInterface.h"
#include "ygz/Viewer.h"
#include "ygz/LoopClosing.h"


// the interface of full system
namespace ygz {
    
    class LoopClosing;
    
    class Tracker;
    
    struct VehicleAttitude;
    
    class System {

    public:
        System(const string &configPath);

        ~System();

        /**
         * insert a stereo image and imu data
         * @param imRectLeft  the rectified left image
         * @param imRectRight the rectified right image
         * @param timestamp
         * @param vimu IMU data before this image
         * @return the Twb pose of the current stereo image
         */
        SE3d AddStereoIMU(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
                          const VecIMU &vimu,double gps_x,double gps_y,double gps_z,bool use_gps, VehicleAttitude va, bool use_atti,double height_input,bool use_height);

        /**
         * insert stereo images
         * @param imRectLeft
         * @param imRectright
         * @param timestamp
         * @return
         */
        SE3d AddStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectright, const double &timestamp);

        // 设置真实轨迹
        void SetGroundTruthTrajectory(map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> &traj);

        void Shutdown();
        

    public:
        
        //shared_ptr<LoopClosing> mpLoopClosing = nullptr;

        shared_ptr<Tracker> mpTracker = nullptr;
        shared_ptr<BackendInterface> mpBackend = nullptr;
        shared_ptr<Viewer> mpViewer = nullptr;
        
        SE3d mCurrentPose;
    };

}

#endif
