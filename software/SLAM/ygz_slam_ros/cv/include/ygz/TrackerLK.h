#ifndef YGZ_TRACKER_LK_H
#define YGZ_TRACKER_LK_H

/**
 * Tracked implemented by LK flow (like VINS)
 */

#include "ygz/Tracker.h"

namespace ygz {

    class TrackerLK : public Tracker {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        TrackerLK(const string &settingFile);

        TrackerLK();

        virtual ~TrackerLK() {}

        // 向Tracker插入一对新的图像，返回估计的位姿 Twb
        // override the ORB tracker
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
        ) override;                      // 自上个帧采集到的IMU
    protected:

        /**
         * 实际的Track函数
         */
        virtual void Track() override;

        /**
         * use lk optical flow to track the features in last frame
         * @return
         */
        virtual bool TrackLastFrame(bool usePoseInfo = false) override;

        /**
         * Track local map using image alignment
         */
        virtual bool TrackLocalMap(int &inliers) override;

        // reset
        virtual void Reset() override;

        // Create stereo matched map points 
        void CreateStereoMapPoints();

        // Clean the old features
        void CleanOldFeatures();

        SE3d mSpeed;       // speed, used to predict the currrent pose
    public:
        void TestTrackerLK();
    };
}

#endif
