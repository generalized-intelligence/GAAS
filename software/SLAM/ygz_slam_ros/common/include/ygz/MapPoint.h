#ifndef YGZ_MAPPOINT_H
#define YGZ_MAPPOINT_H

#include "ygz/Settings.h"
#include "ygz/NumTypes.h"

#include <mutex>

using namespace std;

// 地图点类
// 地图点由后端统一管理，前端在追踪时向后端拉取local map

namespace ygz {

    struct Frame;

    struct Feature;

    struct MapPoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        enum eMapPointState {
            GOOD = 0,    // 正常追踪
            IMMATURE,    // 不成熟的地图点（仅单次2D观测）
            BAD      // 追踪质量差，删除
        };

        /**
         * 从关键帧新建地图点
         * @param kf 参考帧
         * @param indexF 特征点索引
         */
        MapPoint(shared_ptr<Frame> kf, const size_t &indexF);

        // 空构造函数，测试用
        MapPoint() {
            mnId = nNextId++;
        }

        ~MapPoint();

        // get the number of observation
        // 统计有效的观测次数
        int Observations();

        // 获取某一个帧当中的观测
        int GetObsFromKF(shared_ptr<Frame> pKF);

        // 删除某个特定的观测
        bool RemoveObservation( shared_ptr<Frame>& pKF );
        bool RemoveObservation( weak_ptr<Frame>& pKF );

        // add an observation
        void AddObservation(shared_ptr<Frame> pKF, size_t idx);

        void SetRefKF( shared_ptr<Frame> pKF ) {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mpRefKF = pKF;
        }

        // set and test bad flag
        void SetBadFlag();

        // 从观测数据推测这个点是否能够从Immature状态转到good状态
        bool TestGoodFromImmature();

        void SetStatus(eMapPointState state);

        // test bag flag
        bool isBad() {
            return mState == eMapPointState::BAD;
        }

        eMapPointState Status() {
            unique_lock<mutex> lock(mMutexFeatures);
            return mState;
        }

        void IncreaseVisible(int n = 1);

        void IncreaseFound(int n = 1);

        float GetFoundRatio();

        inline int GetFound() {
            return mnFound;
        }

        // 返回最优的描述
        uchar *GetDescriptor() {
            return mDescriptor;
        }

        // 更新法线向量和深度
        void UpdateNormalAndDepth();

        // 更新世界坐标
        void UpdateWorldPos();

        // 计算最优的描述子
        void ComputeDistinctiveDescriptor();

        // accessors, will be locked
        void SetWorldPos(const Vector3d &Pos);

        Vector3d GetWorldPos();

        Vector3d GetNormal();

        // 现有的Ref被移除时，从observation里重新找一个ref
        bool SetAnotherRef();

        // data
        // 尽管定义成public了，但是有些东西被多个线程访问，该上锁请上锁

        long unsigned int mnId = 0; ///< Global ID for MapPoint
        static long unsigned int nNextId; ///< next id

        eMapPointState mState = eMapPointState::GOOD;  ///< 该点状态

        // Position in absolute coordinates
        Vector3d mWorldPos = Vector3d(0, 0, 0); ///< MapPoint在世界坐标系下的坐标，由参考帧的位姿+逆深度得到

        // Keyframes observing the point and associated index in keyframe
        typedef std::map<weak_ptr<Frame>, size_t, std::owner_less<weak_ptr<Frame>>> ObsMap;
        ObsMap mObservations;

        // get all observations
        ObsMap GetObservations();

        // Mean viewing direction
        Vector3d mNormalVector = Vector3d(0, 0, 0);    // 法线 or 观测角

        // Best descriptor to fast matching
        uchar mDescriptor[32]; ///< 通过 ComputeDistinctiveDescriptors() 得到的最优描述子

        // Tracking statistics
        int mnVisible = 1;      // 进入视野的次数
        int mnFound = 1;        // 被匹配到的次数

        bool mbTrackInView = false;  // 是否在Track中已经被匹配到
        float mTrackProjX = -1;           // 预计的投影位置X
        float mTrackProjY = -1;           // 预计的投影位置Y
        int mnTrackScaleLevel = 0;    // 预计的金字塔层级
        float mTrackViewCos = -1.0;   // 视线角
        uchar mGray = 0;              // the gray scale in image

        // 第一次观测到此地图点的关键帧
        weak_ptr<Frame> mpRefKF;

        std::mutex mMutexPos;        // 世界坐标系坐标的锁
        std::mutex mMutexFeatures;   // 特征操作的锁

        // debug only
    public:
        void CheckReprojection();
    };
}


#endif
