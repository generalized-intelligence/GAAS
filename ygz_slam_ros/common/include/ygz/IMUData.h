#ifndef YGZ_IMUDATA_H_
#define YGZ_IMUDATA_H_

#include "ygz/NumTypes.h"

namespace ygz {

    using namespace Eigen;

    struct IMUData {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        IMUData(const double &gx, const double &gy, const double &gz,
                const double &ax, const double &ay, const double &az,
                const double &t) :
                mfGyro(gx, gy, gz), mfAcce(ax, ay, az), mfTimeStamp(t) {}

        IMUData(const Vector3d gyro, const Vector3d &acce, const double &time)
                : mfGyro(gyro), mfAcce(acce), mfTimeStamp(time) {}

        // Raw data of imu
        Vector3d mfGyro;    //gyr data
        Vector3d mfAcce;    //acc data
        double mfTimeStamp;      //timestamp

        // covariance of measurement
        static Matrix3d mfGyrMeasCov;        // 陀螺仪的协方差阵，是方差组成的对角阵
        static Matrix3d mfAccMeasCov;        // 加速度计的协方差阵，是方差组成的对角阵

        // covariance of bias random walk, RW stands for random walk
        static Matrix3d mfGyrBiasRWCov;      // 随机游走的协方差阵
        static Matrix3d mfAccBiasRWCov;      // 加速度计随机游走的协方差阵

    };

    typedef std::vector<IMUData, Eigen::aligned_allocator<IMUData> > VecIMU;
}

#endif // IMUDATA_H
