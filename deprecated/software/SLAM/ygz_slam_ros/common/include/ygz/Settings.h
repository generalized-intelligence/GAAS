#ifndef YGZ_SETTINGS_H_
#define YGZ_SETTINGS_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <stdexcept>
#include <glog/logging.h>

//#include "Thirdparty/DBoW2/DBoW2/FORB.h"
//#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
//#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
//#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"
#include "Thirdparty/DBow3/src/DBoW3.h"

#include "se3.hpp"

// 非常详细的参数调整
// 可以在线调整的，用全局变量；不能的就用常量

namespace ygz {

    typedef DBoW3::Vocabulary //DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
            ORBVocabulary;

    namespace setting {

        // 初始化setting里的变量
        void initSettings();

        // 销毁setting里的变量
        void destroySettings();

        // IMU configs
        // 注意这些都是平方过的
        extern double gyrBiasRw2;   // 陀螺仪的随机游走方差
        extern double accBiasRw2;   // 加速度计随机游走方差

        extern double gyrMeasError2;   // 陀螺仪的测量方差
        extern double accMeasError2;   // 加速度计测量方差

        extern double gravity;           // 重力大小

        extern Eigen::Vector3d biasAccePrior;  // prior accelerator bias

        extern size_t numPyramid;  // 金字塔层数

        // 各层金字塔的缩放倍数
        extern float scalePyramid;      // 金字塔每层至下一层的倍数
        extern float *scaleFactors;     // 各层缩放因子
        extern float *invScaleFactors;  // 倒数
        extern float *levelSigma2;      // 缩放倍数平方
        extern float *invLevelSigma2;   // 平方倒数

        // Frame 的格点设置
        // 使用不同分辨率时，请修改此参数
        extern int FRAME_GRID_SIZE;     // default: 20
        extern int FRAME_GRID_ROWS;     // 480/20
        extern int FRAME_GRID_COLS;     // 752/20

        extern float GridElementWidthInv;
        extern float GridElementHeightInv;

        // Local window configs
        //extern size_t localWindowSize;  // 滑动窗口大小

        // T_B_C        相机到IMU的外参
        // 讲道理的话这个要上锁，后端会覆盖它
        extern Sophus::SE3d TBC;

        // 原始图像分辨率
        extern int imageWidth;
        extern int imageHeight;

        // 外框
        extern int boarder;

        // ORB Extractor 阈值
        extern int initTHFAST;  // 初始门限
        extern int minTHFAST;   // 纹理较差时使用的门限
        extern int extractFeatures;     // 特征点数量
        //const float minShiTomasiScore = 20.0;   // 在计算shi-tomasi分数时的最小阈值
        const float minShiTomasiScore = 10.0;   // 在计算shi-tomasi分数时的最小阈值
        const float featureDistanceGFTT = 30.0;   // in GFTT we fix the feature distance (in pixels), default 30 px

        const int PATCH_SIZE = 31;
        const int HALF_PATCH_SIZE = 15;
        const int EDGE_THRESHOLD = 19;

        // ORBMatcher 的阈值
        //const int TH_LOW = 50;
        //const int TH_HIGH = 100;
        //const int HISTO_LENGTH = 30;

        const int TH_LOW = 70;
        const int TH_HIGH = 200;
        const int HISTO_LENGTH = 30;

        // 双目匹配的阈值
        const float stereoMatchingTolerance = 5.0;  // 双目匹配时，允许在极线上浮动的v轴偏差，默认+-2

        /*** Tracking ***/
        const int minTrackLastFrameFeatures = 10;   // TrackLastFrame时，允许的最小匹配点
        const int minTrackRefKFFeatures = 10;       // TrackRefKF时，允许的最小匹配点
        const int minPoseOptimizationInliers = 10;  // Tracker过程中，位姿优化时的最小点数
        const int minTrackLocalMapInliers = 10;     // TrackLocalMap 过程中，位姿优化时的最小点数
        // 判断地图点是否在视野内时，最远和最近的阈值
        const float minPointDis = 0.5;
        const float maxPointDis = 30;
        //const float maxPointDis = 50;
        const bool useTempMapPoints = true;            // 是否使用中间帧产生的点
        
        
        // Keyframe 相关
        extern double keyframeTimeGapInit;      // 初始化时，两个关键帧之间的时间距离
        extern double keyframeTimeGapTracking;  // 正常跟踪，两个关键帧之间的时间距离
        // 初始化阈值
        const size_t minStereoInitFeatures = 50;    // 初始化时当前帧最小特征点数
        //const size_t minStereoInitFeatures = 10;    // 初始化时当前帧最小特征点数
        //const size_t minValidInitFeatures = 20;     // 初始化时，当前帧有深度值的最少特征点数
        const size_t minValidInitFeatures = 7;     // 初始化时，当前帧有深度值的最少特征点数
        const int minInitKFs = 5;                    // IMU初始化时最少关键帧数量
        extern bool trackerUseHistBalance;      // 是否使用直方图均衡化


        /*** 后端设置 ***/
        extern int numBackendKeyframes/* = 5*/;      // 后端关键帧的数量
        // 新建地图点时，滤掉深度值不在该区间的点
        const float minNewMapPointInvD = 1 / maxPointDis;
        const float maxNewMapPointInvD = 1 / minPointDis;   // D = 0.5m

        // Viewer 设置
        const float cameraSize = 0.1;           // 可视化中相机的大小

    }
}

#endif
