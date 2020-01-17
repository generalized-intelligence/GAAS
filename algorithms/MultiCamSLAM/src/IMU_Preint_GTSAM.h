#ifndef IMU_PREINT_GTSAM
#define IMU_PREINT_GTSAM

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>



namespace mcs
{


using namespace std;
using namespace gtsam;
//#define EIGEN_MAX_STATIC_ALIGN_BYTES 0
//#define Eigen_DONT_VECTORIZE
//using Eigen::Matrix3d;
//using Eigen::Vector3d;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;


struct IMUHelper {
    IMUHelper()
    {
        ;
    }
    IMUHelper(bool do_init,double bias_variance_r = 5.02,double bias_variance_t = 5.0e-3,double velocity_noise_variance = 0.01,double gravity_g = 9.81,
              double acc_variance = 0.0565,double gyro_variance = 0.0565,double bias_acc_variance = 0.00002,double bias_gyro_variance = 0.00002//,
              //Rot3 body_to_imu_R = Rot3(0.036129, -0.998727, 0.035207,
              //                              0.045417, -0.033553, -0.998404,
              //                              0.998315, 0.037670, 0.044147),
              //Point3 body_to_imu_t= Point3(0.03, -0.025, -0.06)
              )
    {
        cout<<"init imu helper!!"<<endl;
        {//stage<1> initialize robust bias kernel.
            auto gaussian = noiseModel::Diagonal::Sigmas(
                        (Vector(6) << Vector3::Constant(bias_variance_r), Vector3::Constant(bias_variance_t))
                        .finished());
            auto huber = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345), gaussian);
            biasNoiseModel = huber;
        }

        {
            auto gaussian = noiseModel::Isotropic::Sigma(3, velocity_noise_variance);
            auto huber = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345), gaussian);

            velocityNoiseModel = huber;
        }

        // expect IMU to be rotated in image space co-ords
        //auto p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(
        //            gtsam::Vector3(0.0, gravity_g, 0.0));

        //PreintegratedCombinedMeasurements::Params params__;
        auto p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(
                             Eigen::Vector3d(0,9.81,0));
        p->accelerometerCovariance =
                I_3x3 * pow(acc_variance, 2.0);  // acc white noise in continuous

        p->integrationCovariance =
                I_3x3 * 1e-9;  // integration uncertainty continuous
//???
        p->gyroscopeCovariance =
                I_3x3 * pow(gyro_variance, 2.0);  // gyro white noise in continuous

        p->biasAccCovariance = I_3x3 * pow(bias_acc_variance, 2.0);  // acc bias in continuous
        p->biasOmegaCovariance =
                I_3x3 * pow(bias_gyro_variance, 2.0);  // gyro bias in continuous
        p->biasAccOmegaInt = gtsam::Matrix::Identity(6, 6) * 1e-5;
//???

        // body to IMU rotation
        Rot3 iRb = Rot3();// = body_to_imu_R;

        // body to IMU translation (meters)
        Point3 iTb = Point3();// = body_to_imu_t;

        // body in this example is the left camera
        p->body_P_sensor = Pose3(iRb, iTb);

        Rot3 prior_rotation = Rot3(I_3x3);
        Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

        Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
        Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

        priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

        prevState = NavState(prior_pose, Vector3(0, 0, 0));
        propState = prevState;
        prevBias = priorImuBias;

        preintegrated = new PreintegratedCombinedMeasurements(p, priorImuBias);
    }

    imuBias::ConstantBias priorImuBias;  // assume zero initial bias
    noiseModel::Robust::shared_ptr velocityNoiseModel;
    noiseModel::Robust::shared_ptr biasNoiseModel;
    NavState prevState;
    NavState propState;
    imuBias::ConstantBias prevBias;
    PreintegratedCombinedMeasurements* preintegrated;
};
/*
struct IMUHelper {
  IMUHelper() {
    {
      auto gaussian = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(5.0e-2), Vector3::Constant(5.0e-3))
              .finished());
      auto huber = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      biasNoiseModel = huber;
    }

    {
      auto gaussian = noiseModel::Isotropic::Sigma(3, 0.01);
      auto huber = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      velocityNoiseModel = huber;
    }

    // expect IMU to be rotated in image space co-ords

    auto p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(
        Vector3(0.0, 9.8, 0.0));

    p->accelerometerCovariance =
        I_3x3 * pow(0.0565, 2.0);  // acc white noise in continuous
    p->integrationCovariance =
        I_3x3 * 1e-9;  // integration uncertainty continuous
    p->gyroscopeCovariance =
        I_3x3 * pow(4.0e-5, 2.0);  // gyro white noise in continuous

    p->biasAccCovariance = I_3x3 * pow(0.00002, 2.0);  // acc bias in continuous
    p->biasOmegaCovariance =
        I_3x3 * pow(0.001, 2.0);  // gyro bias in continuous
    p->biasAccOmegaInt = Matrix::Identity(6, 6) * 1e-5;

    // body to IMU rotation
    Rot3 iRb(0.036129, -0.998727, 0.035207,
             0.045417, -0.033553, -0.998404,
             0.998315, 0.037670, 0.044147);

    // body to IMU translation (meters)
    Point3 iTb(0.03, -0.025, -0.06);

    // body in this example is the left camera
    //p->body_P_sensor = Pose3(iRb, iTb);

    Rot3 prior_rotation = Rot3(I_3x3);
    Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

    Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
    Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

    priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

    prevState = NavState(prior_pose, Vector3(0, 0, 0));
    propState = prevState;
    prevBias = priorImuBias;

    preintegrated = new PreintegratedCombinedMeasurements();//(p, priorImuBias);
  }

  imuBias::ConstantBias priorImuBias;  // assume zero initial bias
  noiseModel::Robust::shared_ptr velocityNoiseModel;
  noiseModel::Robust::shared_ptr biasNoiseModel;
  NavState prevState;
  NavState propState;
  imuBias::ConstantBias prevBias;
  PreintegratedCombinedMeasurements* preintegrated;
};

*/
}
#endif
