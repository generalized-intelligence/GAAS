#ifndef YGZ_VIEWER_H
#define YGZ_VIEWER_H


#include "ygz/Settings.h"
#include "ygz/NumTypes.h"

#include <thread>
#include <mutex>
#include <set>
#include <pangolin/pangolin.h>

// 可视化程序
// 构造后默认调用Run，用AddFrame增加新的帧，用Close关闭
// 或者，用SetBackend关联到后端，那么就仅画出current和后端所有关键帧、地图点

// NOTE MacOS 在创建GUI线程的时候会出现bug，不知道该如何修复

namespace ygz {

    struct Frame;
    struct MapPoint;

    class BackendInterface;

    class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Viewer(bool startViewer = true);

        ~Viewer();

        // 增加一个KeyFrame，将显示该帧的pose和关联的地图点，且将当前帧设为该帧
        void AddFrame(shared_ptr<Frame> frame, bool setToCurrent = true);

        // set a new current frame
        void SetCurrentFrame(shared_ptr<Frame> frame);

        // set tracking status
        void SetTrackStatus(int sta, int trackInliers);

        // 开始
        void Run();

        // 开始且阻塞
        void RunAndSpin();

        // 等待结束
        void WaitToFinish();

        // 结束
        void Close() {
            mbRunning = false;
            mViewerThread.join();
        }

        // Options
        // 是否画出Frame和Point的连线
        void ShowConnection(bool show = true) {
            mbShowConnection = show;
        }

        // 是否画出当前帧的图像
        void ShowCurrentImg(bool show = true) {
            mbShowCurrentImg = show;
        }

        bool IsRunning() const {
            return mbRunning;
        }

        // 画出当前帧图像 (包括特征点)
        cv::Mat DrawImage();

        // 设置真实轨迹
        void SetGTTraj(map<double, Vector3d, std::less<double>, Eigen::aligned_allocator<Vector3d>> &traj) {
            mGTTraj = traj;
            mbShowTrajGT = true;
        }

    private:
        void DrawFrame(shared_ptr<Frame> frame, const Vector3d &color);

        void DrawOrigin();

        void DrawPoints(const Vector3d &color);

        void DrawTrajectory();

        void DrawTrajectoryGT();

        // 以opengl的形式获取当前帧的Twb
        pangolin::OpenGlMatrix GetCurrentGLPose();

        // bias curve log
        pangolin::DataLog mBiasLog;
        std::vector<std::string> mBiasLogLabels;

        // 深度转彩色时用到的函数
        inline Vector3f MakeRedGreen3B(float val) // 0 = red, 10=green, 5=yellow.
        {
            if (val < 0) {
                return Vector3f(0, 0, 1);
            } else if (val < 0.5) {
                return Vector3f(0, 2 * val, 1);
            } else if (val < 1) {
                return Vector3f(0, 1, 1 - 2 * (val - 0.5));
            } else {
                return Vector3f(0, 1, 0);
            }
        }

        bool mbRunning = false;         // 是否正在运行
        std::thread mViewerThread;   // 可视化程序线程
        std::vector<weak_ptr<Frame>> mKeyFrames; // 需要显示的关键帧
        std::set<weak_ptr<MapPoint>, std::owner_less<std::weak_ptr<MapPoint>>> mPoints; // 需要显示的地图点
        shared_ptr<Frame> mCurrentFrame = nullptr; // 当前帧
        VecVector3d mTrajectory;    // 历史轨迹

        map<double, Vector3d, std::less<double>, Eigen::aligned_allocator<Vector3d>> mGTTraj;   // 真实轨迹

        mutex mMutexTrackerStatus;
        int mTrackState = -1;
        int mTrackInliers = -1;

        std::mutex mMutexNewFrame;       // 新加帧时上锁
        shared_ptr<Frame> mpNewFrame =nullptr, mpNewCurrent = nullptr;
        bool mbNewFrameUpdated = false;

        // 选项
        bool mbShowConnection = false;  // 是否画出所有帧观测的地图点
        bool mbShowKFCameras = false;   // 是否画出所有关键帧的相机
        bool mbShowKFGT = false;        // 是否画出所有关键帧的真实Pose
        bool mbShowTrajGT = false;      // 是否画出真实轨迹
        bool mbShowCurrentImg = true;   // 是否显示当前帧图像
        bool mbRecordTrajectory = true; // 是否记录历史轨迹

    public:
        // Debug functions
        void TestViewer();

    };

}

#endif
