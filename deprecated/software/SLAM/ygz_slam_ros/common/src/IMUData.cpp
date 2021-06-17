#include "ygz/Settings.h"
#include "ygz/IMUData.h"


namespace ygz {

    // covariance of measurements
    Matrix3d IMUData::mfGyrMeasCov =
            Matrix3d::Identity() * setting::gyrMeasError2;       // sigma_g * sigma_g / dt, ~6e-6*10
    Matrix3d IMUData::mfAccMeasCov = Matrix3d::Identity() * setting::accMeasError2;

    // covariance of bias random walk
    Matrix3d IMUData::mfGyrBiasRWCov =
            Matrix3d::Identity() * setting::gyrBiasRw2;     // sigma_gw * sigma_gw * dt, ~2e-12
    Matrix3d IMUData::mfAccBiasRWCov =
            Matrix3d::Identity() * setting::accBiasRw2;     // sigma_aw * sigma_aw * dt, ~4.5e-8

}