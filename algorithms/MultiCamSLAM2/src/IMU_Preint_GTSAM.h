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

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cout << "./ISAM2_SmartFactorStereo_IMU [data.txt]\n";
    return 0;
  }

  ifstream in(argv[1]);

  if (!in) {
    cerr << "error opening: " << argv[1] << "\n";
    return 1;
  }

  // Camera parameters
  double fx = 822.37;
  double fy = 822.37;
  double cx = 538.73;
  double cy = 579.10;
  double baseline = 0.372;  // meters

  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  ISAM2 isam(parameters);

  // Create a factor graph
  std::map<size_t, SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;
  NonlinearFactorGraph graph;
  Values initialEstimate;
  IMUHelper imu;

  // Pose prior - at identity
  auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
  graph.emplace_shared<PriorFactor<Pose3>>(X(1), Pose3::identity(),
                                           priorPoseNoise);
  initialEstimate.insert(X(0), Pose3::identity());

  // Bias prior
  graph.add(PriorFactor<imuBias::ConstantBias>(B(1), imu.priorImuBias,
                                               imu.biasNoiseModel));
  initialEstimate.insert(B(0), imu.priorImuBias);

  // Velocity prior - assume stationary
  graph.add(
      PriorFactor<Vector3>(V(1), Vector3(0, 0, 0), imu.velocityNoiseModel));
  initialEstimate.insert(V(0), Vector3(0, 0, 0));

  int lastFrame = 1;
  int frame;

  while (true) {
    char line[1024];

    in.getline(line, sizeof(line));
    stringstream ss(line);
    char type;

    ss >> type;
    ss >> frame;

    if (frame != lastFrame || in.eof()) {
      cout << "Running iSAM for frame: " << lastFrame << "\n";

      initialEstimate.insert(X(lastFrame), Pose3::identity());
      initialEstimate.insert(V(lastFrame), Vector3(0, 0, 0));
      initialEstimate.insert(B(lastFrame), imu.prevBias);

      CombinedImuFactor imuFactor(X(lastFrame - 1), V(lastFrame - 1),
                                  X(lastFrame), V(lastFrame), B(lastFrame - 1),
                                  B(lastFrame), *imu.preintegrated);

      graph.add(imuFactor);

      isam.update(graph, initialEstimate);

      Values currentEstimate = isam.calculateEstimate();

      imu.propState = imu.preintegrated->predict(imu.prevState, imu.prevBias);//这种好像必须配合isam用.不能用传统优化器.
      imu.prevState = NavState(currentEstimate.at<Pose3>(X(lastFrame)),
                               currentEstimate.at<Vector3>(V(lastFrame)));
      imu.prevBias = currentEstimate.at<imuBias::ConstantBias>(B(lastFrame));
      cout<<"imu prevBias:"<<imu.prevBias<<endl;
      imu.preintegrated->resetIntegrationAndSetBias(imu.prevBias);

      graph.resize(0);
      initialEstimate.clear();

      if (in.eof()) {
        break;
      }
    }

    if (type == 'i') {  // Process IMU measurement
      double ax, ay, az;
      double gx, gy, gz;
      double dt = 1 / 800.0;  // IMU at ~800Hz

      ss >> ax;
      ss >> ay;
      ss >> az;

      ss >> gx;
      ss >> gy;
      ss >> gz;

      Vector3 acc(ax, ay, az);
      Vector3 gyr(gx, gy, gz);

      imu.preintegrated->integrateMeasurement(acc, gyr, dt);
    } else if (type == 's') {  // Process stereo measurement
      int landmark;
      double xl, xr, y;

      ss >> landmark;
      ss >> xl;
      ss >> xr;
      ss >> y;

      if (smartFactors.count(landmark) == 0) {
        auto gaussian = noiseModel::Isotropic::Sigma(3, 1.0);

        SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);

        smartFactors[landmark] = SmartStereoProjectionPoseFactor::shared_ptr(
            new SmartStereoProjectionPoseFactor(gaussian, params));
        graph.push_back(smartFactors[landmark]);
      }

      smartFactors[landmark]->add(StereoPoint2(xl, xr, y), X(frame), K);
    } else {
      throw runtime_error("unexpected data type: " + string(1, type));
    }

    lastFrame = frame;
  }

  return 0;
}


*/
}
#endif

/*
 ----------------------------------------------------------------------------


// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>

// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
// #define USE_COMBINED

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

const string output_filename = "imuFactorExampleResults.csv";

// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;

int main(int argc, char* argv[])
{
  string data_filename;
  if (argc < 2) {
    printf("using default CSV file\n");
    data_filename = findExampleDataFile("imuAndGPSdata.csv");
  } else {
    data_filename = argv[1];
  }

  // Set up output file for plotting errors
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

  // Begin parsing the CSV file.  Input the first line for initialization.
  // From there, we'll iterate through the file and we'll preintegrate the IMU
  // or add in the GPS given the input.
  ifstream file(data_filename.c_str());
  string value;

  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();
  getline(file, value, ','); // i
  for (int i=0; i<9; i++) {
    getline(file, value, ',');
    initial_state(i) = atof(value.c_str());
  }
  getline(file, value, '\n');
  initial_state(9) = atof(value.c_str());
  cout << "initial state:\n" << initial_state.transpose() << "\n\n";

  // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
                                         initial_state(4), initial_state(5));
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

#ifdef USE_COMBINED
  imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prior_imu_bias);
#else
  imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
#endif

  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.005;  // The real system has noise, but here, results are nearly
                      // exactly the same, so keeping this for simplicity.

  // All priors have been set up, now iterate through the data file.
  while (file.good()) {

    // Parse out first value
    getline(file, value, ',');
    int type = atoi(value.c_str());

    if (type == 0) { // IMU measurement
      Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
      for (int i=0; i<5; ++i) {
        getline(file, value, ',');
        imu(i) = atof(value.c_str());
      }
      getline(file, value, '\n');
      imu(5) = atof(value.c_str());

      // Adding the IMU preintegration.
      imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

    } else if (type == 1) { // GPS measurement
      Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
      for (int i=0; i<6; ++i) {
        getline(file, value, ',');
        gps(i) = atof(value.c_str());
      }
      getline(file, value, '\n');
      gps(6) = atof(value.c_str());

      correction_count++;

      // Adding IMU factor and GPS factor and optimizing.
#ifdef USE_COMBINED
      PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
      CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                   X(correction_count  ), V(correction_count  ),
                                   B(correction_count-1), B(correction_count  ),
                                   *preint_imu_combined);
      graph->add(imu_factor);
#else
      PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
      ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                           X(correction_count  ), V(correction_count  ),
                           B(correction_count-1),
                           *preint_imu);
      graph->add(imu_factor);
      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
      graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
                                                      B(correction_count  ),
                                                      zero_bias, bias_noise_model));
#endif

      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,1.0);
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),  // N,
                                  gps(1),  // E,
                                  gps(2)), // D,
                           correction_noise);
      graph->add(gps_factor);

      // Now optimize and compare results.
      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);

      LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
      Values result = optimizer.optimize();

      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset the preintegration object.
      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 gtsam_position = prev_state.pose().translation();
      Vector3 position_error = gtsam_position - gps.head<3>();
      current_position_error = position_error.norm();

      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      Quaternion quat_error = gtsam_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3 euler_angle_error(quat_error.x()*2,
                                 quat_error.y()*2,
                                 quat_error.z()*2);
      current_orientation_error = euler_angle_error.norm();

      // display statistics
      cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";

      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2),
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

      output_time += 1.0;

    } else {
      cerr << "ERROR parsing file\n";
      return 1;
    }
  }
  fclose(fp_out);
  cout << "Complete, results written to " << output_filename << "\n\n";;
  return 0;
}

*/


