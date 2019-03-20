#ifndef YGZ_TRACKER_H
#define YGZ_TRACKER_H

#include "ygz/Settings.h"
#include "ygz/NumTypes.h"
#include "ygz/IMUData.h"
#include "ygz/Camera.h"
#include "ygz/Frame.h"
#include "ygz/ORBMatcher.h"
#include "ygz/LoopClosing.h"
#include "ygz/serialization.h"
#include "ygz/MapSerialization.h"

/*
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
*/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/octree.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <deque>
#include <thread>
#include <chrono>

/**
 * The tracker from ORB-SLAM2
 * we use this as an interface and implement a LK tracker from this class
 */


// struct VehicleAttitude
// {
//    Eigen::Quaterniond q;
//    int time_ms;
// };
// 
// typedef std::vector<VehicleAttitude> VecAtti;

namespace ygz {
    
    struct VehicleAttitude
    {
        Eigen::Quaterniond q;
        int time_ms;
    };

    typedef std::vector<VehicleAttitude> VecAtti;

    // forward declare
    class BackendInterface;

    class System;

    class ORBExtractor;

    class Viewer;
    
    class LoopClosing;
    
    class MapSerialization;
    
    class Tracker {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Tracker(const string &settingFile);

        Tracker();

        virtual ~Tracker();

        // 向Tracker插入一对新的图像，返回估计的位姿 Twb
        virtual SE3d InsertStereo(
                const cv::Mat &imRectLeft, const cv::Mat &imRectRight,  // 左右两图
                const double &timestamp,                                // 时间
                const VecIMU &vimu,
		VehicleAttitude va=VehicleAttitude(),
		bool use_atti=false,
                double gps_x = 0,
                double gps_y = 0,
                double gps_z = 0,
                bool use_gps = false,
                double height = 0,
		bool use_height = false
                );                      // 自上个帧采集到的IMU

        enum eTrackingState {
            SYSTEM_NOT_READY = -1,    // 对象创建中
            NO_IMAGES_YET = 0,        // 等待图像读入
            NOT_INITIALIZED = 1,      // 等待初始化（双目需要估计速度、重力方向）
            OK = 2,                   // 正常追踪
            WEAK = 3                  // 视觉弱，追踪依赖IMU，可能不可靠
        };

        eTrackingState GetState() {
            unique_lock<mutex> lock(mMutexState);
            return mState;
        }

        // 重置所有状态量
        virtual void Reset();

        // 求从上一个关键帧到本帧的IMU预积分器
        IMUPreIntegration GetIMUFromLastKF();

        // accessors
        void SetBackEnd(shared_ptr<BackendInterface> backend) {
            mpBackEnd = backend;
        }

        // 获取重力向量
        Vector3d g() const { return mgWorld; }

        // 设置重力
        void SetGravity(const Vector3d &g) { mgWorld = g; }

        void SetViewer(shared_ptr<Viewer> viewer) {
            mpViewer = viewer;
        }

        void SetCamera(shared_ptr<CameraParam> camera) {
            mpCam = camera;
        }

        void SetPureVisionMode(bool pureVision = true) {
            mbVisionOnlyMode = true;
        }
        
        void setLoopClosing(shared_ptr<LoopClosing> mLC);

    protected:
        // inner functions
        /**
         * 实际的Track函数
         */
        virtual void Track();

        /**
         * 与lastFrame的比较，如果有pose信息，则使用投影模型，否则使用特征匹配
         * @param[in] usePoseInfomation 是否使用current里已有的pose信息
         * @return true if success
         */
        virtual bool TrackLastFrame(bool usePoseInfomation = true);

        // 与 Reference KF 的比较
        virtual bool TrackReferenceKF(bool usePoseInfomation = true);

        // 与局部地图的比较
        virtual bool TrackLocalMap(int &inliers);

        // 判断是否需要关键帧
        bool NeedNewKeyFrame(const int &trackinliers);

        // 插入关键帧
        virtual void InsertKeyFrame();

        // 用 IMU 数据预计当前帧的 Pose
        void PredictCurrentPose();

        /**
         * 优化当前帧的 Pose
         * 会写入优化的outlier信息
         * @return inlier数量
         */
        int OptimizeCurrentPose();

        // 带着IMU约束优化
        int OptimizeCurrentPoseWithIMU();

        /**
         * 不使用g2o的更快的优化
         * @return
         */
        int OptimizeCurrentPoseFaster();

        // 不带IMU约束优化
        int OptimizeCurrentPoseWithoutIMU();

        // 更新上一个关键帧的信息
        void UpdateLastFrame();

        // 双目初始化
        // 将所有invDepth大于零的特征点转换为地图点
        bool StereoInitialization();

        // IMU 初始化
        bool IMUInitialization();

        // Optimization in IMU init
        // 用关键帧估计IMU中的gyro bias
        Vector3d IMUInitEstBg(const std::deque<shared_ptr<Frame>> &vpKFs);
        


    protected:

        mutex mMutexState;
        eTrackingState mState = SYSTEM_NOT_READY;

        VecIMU mvIMUSinceLastKF;  // 从上一个帧直到当前帧的IMU数据

        // TODO 用不用智能指针？
        shared_ptr<Frame> mpCurrentFrame = nullptr;  // 当前帧
        shared_ptr<Frame> mpLastFrame = nullptr;     // 上一个帧
        shared_ptr<Frame> mpLastKeyFrame = nullptr;  // 上一个关键帧
        shared_ptr<CameraParam> mpCam = nullptr;     // 相机内参

        // System
        shared_ptr<System> mpSystem = nullptr;

        // BackEnd
        shared_ptr<BackendInterface> mpBackEnd = nullptr;

        Vector3d mgWorld = Vector3d(0, 0, setting::gravity);   // 世界坐标系下的重力

        // ORB Extractor
        shared_ptr<ORBExtractor> mpExtractor = nullptr;

        // ORB Matcher
        shared_ptr<ORBMatcher> mpMatcher = nullptr;

        // Viewer
        shared_ptr<Viewer> mpViewer = nullptr;
        
        // loopclosing
        shared_ptr<LoopClosing> mpLoopClosing = nullptr;
        
        // MapSerialization
        shared_ptr<MapSerialization> mpMapSerialization = nullptr;
        
        
        int mTrackInliersCnt = 0;
        double mGPSx;
        double mGPSy;
        double mGPSz;
        bool mUseGPS;
        VehicleAttitude mVA;
        bool use_mVA;
        double mHeight;
        bool use_mHeight;
        
        bool mBuildObstacleMap;
        
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;

        float resolution = 128.0;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* pOctree;
        std::map<int,shared_ptr<FrameLight>> id_to_frame_map;

        int frame_id;
        std::thread obstacle_map_thread;
        
        int frame_to_build_count;
        std::mutex frame_to_build_count_mutex;
        
        shared_ptr<octomap::OcTree> pObstacle_tree;//only accessed by obstacle_building thread,don't need any mutex.
        public:
        void do_obstacle_map_building_main();
        
    public:
        // 测试部分
        void TestStereoInit();  // 测试双目初始化
        void TestPureVision();  // 测试纯视觉追踪

        bool mbVisionOnlyMode = false;  // 仅视觉模式？
        void buildCloudAndOctree();
    };

}

#endif
