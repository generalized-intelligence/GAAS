#ifndef YGZ_BACKEND_SLIDING_WINDOW_G2O_H
#define YGZ_BACKEND_SLIDING_WINDOW_G2O_H

#include "ygz/NumTypes.h"
#include "ygz/Settings.h"
#include "ygz/BackendInterface.h"

#include <deque>
#include <set>


/**
 * 用G2O实现的一个滑窗后端
 * （其实G2O并不适合处理滑窗……）
 * 没有Marg和FEJ，啦啦啦
 * 这货后期要改成单独一个线程用的
 */

namespace ygz {

    // forward declare
    class Tracker;

    class BackendSlidingWindowG2O : public BackendInterface {

    public:
        BackendSlidingWindowG2O(shared_ptr<Tracker> tracker) : BackendInterface(), mpTracker(tracker) {
            mtBackendMainLoop = thread(&BackendSlidingWindowG2O::MainLoop, this);
        }

        virtual ~BackendSlidingWindowG2O() {}

        // the main loop thread
        void MainLoop();

        // 插入新关键帧
        virtual int InsertKeyFrame(shared_ptr<Frame> newKF) override;

        // 查询后端状态
        virtual bool IsBusy() override;

        // 关闭后端
        virtual void Shutdown() override;

        // 获取局部地图（前端追踪用）
        virtual std::set<shared_ptr<MapPoint>> GetLocalMap() override;

        // 获取所有的关键帧
        virtual std::deque<shared_ptr<Frame>> GetAllKF() override {
            unique_lock<mutex> lock(mMutexKFs);
            return mpKFs;
        }

        virtual void Reset() override;

        virtual void CallLocalBA() override;

        virtual void OptimizeCurrent(shared_ptr<Frame> current) override;

        bool Stop() {
            unique_lock<mutex> lock(mMutexStop);
            if (mbStopRequested && !mbNotStop) {
                mbStopped = true;
                LOG(INFO) << "Local Mapping STOP" << endl;
                return true;
            }
            return false;
        }

        // 请求停止主线程
        void RequestStop() {
            unique_lock<mutex> lock(mMutexStop);
            mbStopRequested = true;
            unique_lock<mutex> lock2(mMutexNewKFs);
            mbAbortBA = true;
        }

        bool isStopped() {
            unique_lock<mutex> lock(mMutexStop);
            return mbStopped;
        }

        bool StopRequested() {
            unique_lock<mutex> lock(mMutexStop);
            return mbStopRequested;
        }

        // 释放所有数据
        void Release() {
            unique_lock<mutex> lock(mMutexStop);
            unique_lock<mutex> lock2(mMutexFinish);
            if (mbFinished)
                return;
            mbStopped = false;
            mbStopRequested = false;
            mpNewKFs.clear();
            mpKFs.clear();
            mpPoints.clear();
            mpCurrent = nullptr;
            LOG(INFO) << "Local Mapping RELEASE" << endl;
        }

        bool SetNotStop(bool flag) {
            unique_lock<mutex> lock(mMutexStop);
            if (flag && mbStopped)
                return false;
            mbNotStop = flag;
            return true;
        }

        void InterruptBA() {
            mbAbortBA = true;
        }

        void RequestReset() {
            {
                unique_lock<mutex> lock(mMutexReset);
                mbResetRequested = true;
                mbAbortBA = true;
            }

            while (1) {
                {
                    unique_lock<mutex> lock2(mMutexReset);
                    if (!mbResetRequested)
                        break;
                }
                usleep(3000);
            }
            mbAbortBA = false;
        }

        void ResetIfRequested() {
            unique_lock<mutex> lock(mMutexReset);
            if (mbResetRequested) {
                mpNewKFs.clear();
                mbResetRequested = false;
            }
        }

        void RequestFinish() {
            unique_lock<mutex> lock(mMutexFinish);
            mbFinishRequested = true;
        }

        bool CheckFinish() {
            unique_lock<mutex> lock(mMutexFinish);
            return mbFinishRequested;
        }

        void SetFinish() {
            mbAbortBA = true;
            unique_lock<mutex> lock(mMutexFinish);
            mbFinished = true;
            unique_lock<mutex> lock2(mMutexStop);
            mbStopped = true;

            mtBackendMainLoop.join();
        }

        bool isFinished() {
            unique_lock<mutex> lock(mMutexFinish);
            return mbFinished;
        }

        //  设置是否可以接受新关键帧
        void SetAcceptKeyFrames(bool flag) {
            unique_lock<mutex> lock(mMutexAccept);
            mbAcceptKeyFrames = flag;
        }

        // 检查关键帧队列中是否有新关键帧
        bool CheckNewKeyFrames() {
            unique_lock<mutex> lock(mMutexNewKFs);
            return !mpNewKFs.empty();
        }


    private: // 一些中间函数
        // 从关键帧队列中取出最新一帧并进行处理
        void ProcessNewKeyFrame();

        // 创建新地图点
        int CreateNewMapPoints();

        // 删除第idx个帧
        void DeleteKF(int idx);

        // 清理地图点，若ref失效则重新找ref
        int CleanMapPoint();

        // Local BA，分带IMU和不带IMU两个版本。Tracker未初始化时用不带IMU的，初始化之后用带IMU的
        void LocalBAWithIMU(bool verbose = false);

        void LocalBAWithoutIMU(bool verbose = false);

        void LocalBAXYZWithoutIMU(bool verbose = false);

        void LocalBAXYZWithIMU(bool verbose = false);

        // 计算两个帧之间的Funcdamental
        Matrix3d ComputeF12(shared_ptr<Frame> f1, shared_ptr<Frame> f2);

    private:
        shared_ptr<Tracker> mpTracker = nullptr; // Tracker指针，需要向Tracker通报一些状态
        shared_ptr<Frame> mpCurrent = nullptr;      // 当前正处理的帧

        bool mbFirstCall = true;

        std::deque<shared_ptr<Frame>> mpKFs;   // 关键帧队列，会保持一定长度
        std::deque<shared_ptr<Frame>> mpNewKFs;    // 由前端插入的新关键帧队列
        std::set<shared_ptr<MapPoint>> mpPoints;   // 局部地图点，同样会操持一定长度

        // mutex
        std::mutex mMutexReset;
        std::mutex mMutexFinish;
        std::mutex mMutexNewKFs;
        std::mutex mMutexStop;
        std::mutex mMutexAccept;

        std::mutex mMutexKFs;
        std::mutex mMutexPoints;


        // state variables
        bool mbAbortBA = false;
        bool mbStopped = false;
        bool mbStopRequested = false;
        bool mbNotStop = false;
        bool mbAcceptKeyFrames = false;
        bool mbFinishRequested = false;
        bool mbFinished = false;
        bool mbResetRequested = false;

        // main thread
        std::thread mtBackendMainLoop;

    public:
        // 测试函数
        void testLocalBA(); // Local BA的测试
        void testLocalBAIMU(); // Local BA with IMU的测试

        // Debug only
        // 输出所有KF的信息
        void PrintKFInfo();
    };
}

#endif
