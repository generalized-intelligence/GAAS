#include "ygz/Settings.h"

namespace ygz {

    namespace setting {
        // IMU related
        // *10, *100 是王京同学想出来的
        /**
         * For EuRoc dataset, according to V1_01_easy/imu0/sensor.yaml
         * The params:
         * sigma_g: 1.6968e-4       rad / s / sqrt(Hz)
         * sigma_gw: 1.9393e-5      rad / s^2 / sqrt(Hz)
         * sigma_a: 2.0e-3          m / s^2 / sqrt(Hz)
         * sigma_aw: 3.0e-3         m / s^3 / sqrt(Hz)
         */
        /*
        double gyrBiasRw2 = 2.0e-5 * 2.0e-5  * 100 ;   // 陀螺仪的随机游走方差
        double accBiasRw2 = 5.0e-3 * 5.0e-3  * 100 ;   // 加速度计随机游走方差
        double gyrMeasError2 = 1.7e-4 * 1.7e-4 / 0.005  ;   // 陀螺仪的测量方差
        double accMeasError2 = 2.0e-3 * 2.0e-3 / 0.005  ;   // 加速度计测量方差
         */

        double gyrBiasRw2 = 2.0e-5 * 2.0e-5;   // 陀螺仪的随机游走方差
        double accBiasRw2 = 5.0e-3 * 5.0e-3;   // 加速度计随机游走方差
        double gyrMeasError2 = 1.7e-1 * 1.7e-1;   // 陀螺仪的测量方差
        double accMeasError2 = 2.0e1 * 2.0e1;   // 加速度计测量方差
        double gravity = 9.810;

        size_t numPyramid = 5;
        float scalePyramid = 2;      // 金字塔每层至下一层的倍数
        float *scaleFactors = nullptr;     // 各层缩放因子
        float *invScaleFactors = nullptr;  // 倒数
        float *levelSigma2 = nullptr;      // 缩放倍数平方
        float *invLevelSigma2 = nullptr;   // 平方倒数

        Eigen::Matrix3d Rbc = [] {
            Eigen::Matrix3d mat;
            mat << 0.0148655429818, -0.999880929698, 0.00414029679422,
                    0.999557249008, 0.0149672133247, 0.025715529948,
                    -0.0257744366974, 0.00375618835797, 0.999660727178;
            return mat;
        }();
        Eigen::Vector3d tbc(-0.0216401454975, -0.064676986768, 0.00981073058949);
        Sophus::SE3d TBC(Rbc, tbc);

        int FRAME_GRID_SIZE = 40;
        int FRAME_GRID_ROWS = 0;
        int FRAME_GRID_COLS = 0;

        Eigen::Vector3d biasAccePrior(-0.025, 0.136, 0.075);

        // 原始图像分辨率
        int imageWidth = 752;
        int imageHeight = 480;

        // 外框
        int boarder = 20;

        // ORB Matcher 的阈值
        int ORBMatcher_TH_HIGH = 100;
        int ORBMatcher_TH_LOW = 50;

        // ORB Extractor 阈值
        int initTHFAST = 20;  // 初始门限
        int minTHFAST = 5;   // 纹理较差时使用的门限
        int extractFeatures = 100;     // 特征点数量

        float GridElementWidthInv;
        float GridElementHeightInv;

        bool trackerUseHistBalance = true;     // Tracker是否要计算gray histogram balance

        int numBackendKeyframes = 10;
        double keyframeTimeGapInit = 0.5;      // 初始化时，两个关键帧之间的时间距离
        double keyframeTimeGapTracking = 0.5;  // 正常跟踪，两个关键帧之间的时间距离

        void initSettings() {
            // compute the scale factors in each frame
            scaleFactors = new float[numPyramid];
            levelSigma2 = new float[numPyramid];
            scaleFactors[0] = 1.0f;
            levelSigma2[0] = 1.0f;
            for (size_t i = 1; i < numPyramid; i++) {
                scaleFactors[i] = scaleFactors[i - 1] * scalePyramid;
                levelSigma2[i] = scaleFactors[i] * scaleFactors[i];
            }

            invScaleFactors = new float[numPyramid];
            invLevelSigma2 = new float[numPyramid];
            for (size_t i = 0; i < numPyramid; i++) {
                invScaleFactors[i] = 1.0f / scaleFactors[i];
                invLevelSigma2[i] = 1.0f / levelSigma2[i];
            }

            FRAME_GRID_ROWS = ceil(imageHeight / FRAME_GRID_SIZE);
            FRAME_GRID_COLS = ceil(imageWidth / FRAME_GRID_SIZE);
            // GridElementWidthInv = static_cast<float> ( FRAME_GRID_COLS ) / static_cast<float> ( imageWidth );
            // GridElementHeightInv = static_cast<float> ( FRAME_GRID_ROWS ) / static_cast<float> ( imageHeight );
            GridElementWidthInv = 1.0 / float(FRAME_GRID_SIZE);
            GridElementHeightInv = 1.0 / float(FRAME_GRID_SIZE);
        }

        void destroySettings() {
            delete[] scaleFactors;
            delete[] levelSigma2;
            delete[] invScaleFactors;
            delete[] invLevelSigma2;
        }

    }
}
