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


#include <vector>
#include <glog/logging.h>

#include "offline_dataset.h"
//#define USE_COMBINED
using namespace std;
using namespace gtsam;

// Shorthand for velocity and pose variables
using symbol_shorthand::V;
using symbol_shorthand::X;
using symbol_shorthand::B;
using Eigen::Vector3d;

class SequentialIMUPreintegrator
{
public:
    typedef imuBias::ConstantBias IMUBiasType;
    std::shared_ptr<PreintegrationType> imu_preintegrated_;
    geometry_msgs::PoseStamped prev_pose;
    vector<sensor_msgs::Imu> imu_vec;
    ros::Time prev_stamp;
    Vector3d initial_velocity = Vector3d(0,0,0);
    //Vector3d current_velocity = Vector3d(0,0,0);
    bool ever_init = false;

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p;
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

    NavState prev_state,prop_state;
    imuBias::ConstantBias prev_bias;

    std::shared_ptr<NonlinearFactorGraph> graph;
    Values initial_values;

    long long int curr_pose_index = 0;//决定速度估计的noise.
    double velocity_uncertainty = 1.0;// m/s
    void init(geometry_msgs::PoseStamped pose,Eigen::Vector3d* initial_velocity_pvec=nullptr,imuBias::ConstantBias* initial_IMU_bias=nullptr)
    {

        this->prev_pose = pose;
        prev_stamp = prev_pose.header.stamp;
        if(initial_velocity_pvec!=nullptr)
        {
            this->initial_velocity = *initial_velocity_pvec;
        }
        else
        {
            this->initial_velocity = Vector3(0,0,0); //zero initial velocity assumption.
        }
        if(initial_IMU_bias != nullptr)
        {
            prior_imu_bias = *initial_IMU_bias;
        }
        else
        {
            imuBias::ConstantBias zero_imu_bias;
            prior_imu_bias = zero_imu_bias;
        }

        //init const variables
        double accel_noise_sigma = 0.002;
        double gyro_noise_sigma = 0.0006;
        double accel_bias_rw_sigma = 0.00002;
        double gyro_bias_rw_sigma = 0.000003;
        Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
        Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
        Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
        Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
        Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
        Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

        p = PreintegratedCombinedMeasurements::Params::MakeSharedU();
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

        //init variables
#ifdef USE_COMBINED
        imu_preintegrated_ =  std::shared_ptr<PreintegrationType>(new PreintegratedCombinedMeasurements(p, prior_imu_bias));
#else
        imu_preintegrated_ = std::shared_ptr<PreintegrationType>(new PreintegratedImuMeasurements(p, prior_imu_bias));
#endif
        prev_state = NavState(poseFromMsg(pose), initial_velocity);
        prop_state = prev_state;

        graph = std::shared_ptr<NonlinearFactorGraph>(new(NonlinearFactorGraph));
        ever_init = true;
    }
    bool do_prop(const sensor_msgs::Imu& m,geometry_msgs::PoseStamped& output_pose_estimation)
    {
        if(!ever_init||m.header.stamp<prev_stamp)
        {
            prev_stamp = m.header.stamp;
            return false;
        }
        imu_vec.push_back(m);//放入队列。
        double dt = (m.header.stamp - prev_stamp).toSec();
        if(dt<=0)
        {
            LOG(INFO)<<"warning dt<=0!"<<endl;
            LOG(INFO)<<"dt:"<<dt<<endl;
            dt = 0.0001;
        }
        if(dt>0.05)//50ms,20hz.
        {
            dt=0.05;//maximux tolerance.
        }
        prev_stamp = m.header.stamp;
        Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
        imu(0) = m.linear_acceleration.x;
        imu(1) = m.linear_acceleration.y;
        imu(2) = m.linear_acceleration.z;

        imu(3) = m.angular_velocity.x;
        imu(4) = m.angular_velocity.y;
        imu(5) = m.angular_velocity.z;

        prev_stamp = m.header.stamp;
        imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

        prop_state = imu_preintegrated_->predict(prev_state,prev_bias);
        output_pose_estimation.header.stamp = m.header.stamp;
        output_pose_estimation.header.frame_id="imu";
        Point3 curr_position = prop_state.position();
        Quaternion curr_quat = prop_state.quaternion();
        output_pose_estimation.pose.position.x = curr_position.x();
        output_pose_estimation.pose.position.y = curr_position.y();
        output_pose_estimation.pose.position.z = curr_position.z();
        output_pose_estimation.pose.orientation.w = curr_quat.w();
        output_pose_estimation.pose.orientation.x = curr_quat.x();
        output_pose_estimation.pose.orientation.y = curr_quat.y();
        output_pose_estimation.pose.orientation.z = curr_quat.z();

        return true;
    }
    bool do_optimize(const geometry_msgs::PoseStamped& curr_pose,Vector3* pVelocityOut=nullptr,imuBias::ConstantBias* pIMUBiasOut=nullptr)//,vector<sensor_msgs::Imu>* pimu_vec = nullptr)
    {
        prev_stamp = curr_pose.header.stamp;
//        if(pimu_vec!=nullptr)
//        {
//            this->imu_vec = *pimu_vec;
//        }
        if(imu_vec.size()<2)
        {
            LOG(WARNING)<<"imu vec too short!size:"<<imu_vec.size()<<endl;
            imu_vec.clear();
            prev_pose = curr_pose;
            return false;//没有imu，无法优化imu factor.
        }
        Vector3 gt_vel = (poseFromMsg(curr_pose).translation().vector() - poseFromMsg(prev_pose).translation().vector())/(curr_pose.header.stamp - prev_pose.header.stamp).toSec();

        //add pose_factor
        noiseModel::Diagonal::shared_ptr ndt_pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.0001, 0.00001, 0.00001, 0.00005, 0.00005, 0.00005).finished()); // rad,rad,rad,m, m, m
        //noiseModel::Diagonal::shared_ptr ndt_pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m
        noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,velocity_uncertainty); // m/s //unknown speed.
//        if(curr_pose_index<6)//16,8,4,2,1,0.5
//        {
//            velocity_uncertainty/=1.1;
//        }
        //noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
        noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-2);
        std::shared_ptr<NonlinearFactorGraph> new_graph = std::shared_ptr<NonlinearFactorGraph>(new NonlinearFactorGraph);
        new_graph->add(PriorFactor<Pose3>(X(0),poseFromMsg(prev_pose),ndt_pose_noise_model));
        new_graph->add(PriorFactor<Pose3>(X(1),poseFromMsg(curr_pose),ndt_pose_noise_model));

        //add pose initial estimation
        initial_values.clear();
        prop_state = imu_preintegrated_->predict(prev_state, prev_bias);

        initial_values.insert(X(0), poseFromMsg(prev_pose));
        initial_values.insert(V(0), prev_state.v());//TODO:尝试差分位置求速度。
        //initial_values.insert(V(0), gt_vel);//gt_vel
        initial_values.insert(B(0), prev_bias);
        initial_values.insert(X(1), poseFromMsg(curr_pose));//prop_state.pose());
        initial_values.insert(V(1), prop_state.v());
        //initial_values.insert(V(1), gt_vel);
        //initial_values.insert(V(1), current_velocity);
        initial_values.insert(B(1), prev_bias);



        //add imu_factor

#ifdef USE_COMBINED
        std::shared_ptr<PreintegratedCombinedMeasurements> preint_imu_combined = dynamic_pointer_cast<PreintegratedCombinedMeasurements>(imu_preintegrated_);
        CombinedImuFactor imu_factor(X(0), V(0),
                                     X(1), V(1),
                                     B(0), B(1),
                                     *preint_imu_combined);
        new_graph->add(imu_factor);
#else
        std::shared_ptr<PreintegratedImuMeasurements> preint_imu = dynamic_pointer_cast<PreintegratedImuMeasurements>(imu_preintegrated_);
        ImuFactor imu_factor(X(0), V(0),
                             X(1), V(1),
                             B(0),
                             *preint_imu);
        new_graph->add(imu_factor);
        imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        new_graph->add(BetweenFactor<imuBias::ConstantBias>(B(0),
                                                            B(1),
                                                            zero_bias, bias_noise_model));

#endif
        Pose3 propagated_pose = prop_state.pose();
        Vector3 prop_position = propagated_pose.translation();


        //gtsam::LevenbergMarquardtParams params_lm;
        //params_lm.setVerbosity("ERROR");
        //params_lm.setMaxIterations(30);
        //params_lm.setLinearSolverType("MULTIFRONTAL_CHOLESKY");

        LevenbergMarquardtOptimizer optimizer(*new_graph, initial_values);
        //LevenbergMarquardtOptimizer optimizer(*new_graph, initial_values,params_lm);
        Values result;
        LOG(INFO)<<"before optimization..."<<endl;
        result = optimizer.optimize();//replace global optimization with local ones.
        LOG(INFO)<<"after "<<optimizer.getInnerIterations()<<" times optimization..."<<endl;
        // Overwrite the beginning of the preintegration for the next step.
        prev_state = NavState(poseFromMsg(curr_pose),
                              result.at<Vector3>(V(1)));
        prop_state = prev_state;
        prev_bias = result.at<imuBias::ConstantBias>(B(1));

        // Reset the preintegration object.


        // Print out the position and orientation error for comparison.
        imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

        auto curr_ndt_pose = poseFromMsg(curr_pose);
        Vector3 ndt_position = curr_ndt_pose.translation().vector();
        Vector3 position_error = prop_position - ndt_position;
        double current_position_error = position_error.norm();

        Quaternion prop_quat = propagated_pose.rotation().toQuaternion();
        Quaternion ndt_quat = curr_ndt_pose.rotation().toQuaternion();
        Quaternion quat_error = prop_quat * ndt_quat.inverse();
        quat_error.normalize();
        Vector3 euler_angle_error(quat_error.x()*2,
                                  quat_error.y()*2,
                                  quat_error.z()*2);
        double current_orientation_error = euler_angle_error.norm();

        //print result.
        LOG(INFO)<<"Between Pose dt:"<<(curr_pose.header.stamp-prev_pose.header.stamp).toSec()<<"s."<<endl<<endl;
        printf("imu prop: %f,%f,%f,%f,%f,%f,%f,\ngt_val  : %f,%f,%f,%f,%f,%f,%f\n",
               prop_position(0), prop_position(1), prop_position(2),
               prop_quat.x(), prop_quat.y(), prop_quat.z(), prop_quat.w(),
               ndt_position(0), ndt_position(1), ndt_position(2),
               ndt_quat.x(), ndt_quat.y(), ndt_quat.z(), ndt_quat.w());
        LOG(INFO)<<"Imu vector size:"<<imu_vec.size()<<endl;
        LOG(INFO) <<"Position error:"<<current_position_error<<"m;rot error:"<<(current_orientation_error*180)/3.14159<<" deg."<<endl;
        LOG(INFO) <<"Speed:"<<this->prev_state.v().norm()<<"m/s."<<endl;
        LOG(INFO) <<"Velocity:"<<this->prev_state.v()<<endl;

        LOG(INFO) <<"gt_velo:"<<gt_vel<<endl;
        LOG(INFO) <<"velo residual:"<<(gt_vel-prev_state.v()).norm()<<" m/s."<<endl;
        LOG(INFO) <<"bias:"<<prev_bias<<endl;
        //prepare next generation
        LOG(INFO)<<"--------------------------------------------------------------"<<endl<<endl;

        if(pVelocityOut!=nullptr)
        {
            *pVelocityOut=prev_state.v();
        }
        if(pIMUBiasOut!=nullptr)
        {
            *pIMUBiasOut = prev_bias;
        }

        imu_vec.clear();
        prev_pose = curr_pose;
        curr_pose_index++;
        return true;
    }
//private:
    Pose3 poseFromMsg(const geometry_msgs::PoseStamped& pose)
    {
        Rot3 _rotation = Rot3::Quaternion(pose.pose.orientation.w, pose.pose.orientation.x,
                                          pose.pose.orientation.y, pose.pose.orientation.z);
        Point3 _point(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        Pose3 _pose(_rotation, _point);
        return _pose;
    }

};

int run_all_pipeline(int argc, char* argv[],SequenceDataset& sd)
{
    geometry_msgs::PoseStamped init_pose,pose;
    vector<sensor_msgs::Imu> imu_vec;
    while(true)
    {
        if(!sd.next(imu_vec,init_pose))
        {
            LOG(INFO)<<"ERROR:no valid data."<<endl;
            break;
        }
        if(imu_vec.size())
        {
            break;
        }
    }
    SequentialIMUPreintegrator sip;
    sip.init(init_pose);
    while(true)
    {
        geometry_msgs::PoseStamped pose_prop;
        for(auto u:imu_vec)
        {
            sip.do_prop(u,pose_prop);
        }

        if(!sd.next(imu_vec,pose))
        {
            LOG(INFO)<<"end."<<endl;
            return 0;
        }
        sip.do_optimize(pose);
    }
}

