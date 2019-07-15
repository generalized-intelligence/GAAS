/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 4, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include "glog/logging.h"
#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

double sinc_test(double x){
	if(fabs(x)>1e-10) {
		return sin(x)/x;
	}
	else{
		static const double c_2=1.0/6.0;
		static const double c_4=1.0/120.0;
		static const double c_6=1.0/5040.0;
		const double x_2 = x*x;
		const double x_4 = x_2*x_2;
		const double x_6 = x_2*x_2*x_2;
		return 1.0 - c_2*x_2 + c_4*x_4 - c_6*x_6;
	}
}

const double jacobianTolerance = 1.0e-3;

TEST(okvisTestSuite, ImuError){
	// initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

	// Build the problem.
	::ceres::Problem problem;

  // check errors
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

	// set the imu parameters
	okvis::ImuParameters imuParameters;
	imuParameters.a0.setZero();
	imuParameters.g = 9.81;
	imuParameters.a_max = 1000.0;
	imuParameters.g_max = 1000.0;
	imuParameters.rate = 1000; // 1 kHz
	imuParameters.sigma_g_c = 6.0e-4;
	imuParameters.sigma_a_c = 2.0e-3;
	imuParameters.sigma_gw_c = 3.0e-6;
	imuParameters.sigma_aw_c = 2.0e-5;
	imuParameters.tau = 3600.0;

	// generate random motion
	const double w_omega_S_x = Eigen::internal::random(0.1,10.0); // circular frequency
	const double w_omega_S_y = Eigen::internal::random(0.1,10.0); // circular frequency
	const double w_omega_S_z = Eigen::internal::random(0.1,10.0); // circular frequency
	const double p_omega_S_x = Eigen::internal::random(0.0,M_PI); // phase
	const double p_omega_S_y = Eigen::internal::random(0.0,M_PI); // phase
	const double p_omega_S_z = Eigen::internal::random(0.0,M_PI); // phase
	const double m_omega_S_x = Eigen::internal::random(0.1,1.0); // magnitude
	const double m_omega_S_y = Eigen::internal::random(0.1,1.0); // magnitude
	const double m_omega_S_z = Eigen::internal::random(0.1,1.0); // magnitude
	const double w_a_W_x = Eigen::internal::random(0.1,10.0);
	const double w_a_W_y = Eigen::internal::random(0.1,10.0);
	const double w_a_W_z = Eigen::internal::random(0.1,10.0);
	const double p_a_W_x = Eigen::internal::random(0.1,M_PI);
	const double p_a_W_y = Eigen::internal::random(0.1,M_PI);
	const double p_a_W_z = Eigen::internal::random(0.1,M_PI);
	const double m_a_W_x = Eigen::internal::random(0.1,10.0);
	const double m_a_W_y = Eigen::internal::random(0.1,10.0);
	const double m_a_W_z = Eigen::internal::random(0.1,10.0);

	// generate randomized measurements - duration 10 seconds
	const double duration = 1.0;
	okvis::ImuMeasurementDeque imuMeasurements;
	okvis::kinematics::Transformation T_WS;
	//T_WS.setRandom();

	// time increment
	const double dt=1.0/double(imuParameters.rate); // time discretization

	// states
	Eigen::Quaterniond q=T_WS.q();
	Eigen::Vector3d r=T_WS.r();
	okvis::SpeedAndBias speedAndBias;
	speedAndBias.setZero();
	Eigen::Vector3d v=speedAndBias.head<3>();

	// start
	okvis::kinematics::Transformation T_WS_0;
	okvis::SpeedAndBias speedAndBias_0;
	okvis::Time t_0;

	// end
	okvis::kinematics::Transformation T_WS_1;
	okvis::SpeedAndBias speedAndBias_1;
	okvis::Time t_1;

	for(size_t i=0; i<size_t(duration*imuParameters.rate); ++i){
	  double time = double(i)/imuParameters.rate;
	  if (i==10){ // set this as starting pose
		  T_WS_0 = T_WS;
		  speedAndBias_0=speedAndBias;
		  t_0=okvis::Time(time);
	  }
	  if (i==size_t(duration*imuParameters.rate)-10){ // set this as starting pose
		  T_WS_1 = T_WS;
		  speedAndBias_1=speedAndBias;
		  t_1=okvis::Time(time);
	  }

	  Eigen::Vector3d omega_S(m_omega_S_x*sin(w_omega_S_x*time+p_omega_S_x),
			  m_omega_S_y*sin(w_omega_S_y*time+p_omega_S_y),
			  m_omega_S_z*sin(w_omega_S_z*time+p_omega_S_z));
	  Eigen::Vector3d a_W(m_a_W_x*sin(w_a_W_x*time+p_a_W_x),
				  m_a_W_y*sin(w_a_W_y*time+p_a_W_y),
				  m_a_W_z*sin(w_a_W_z*time+p_a_W_z));

	  //omega_S.setZero();
	  //a_W.setZero();

	  Eigen::Quaterniond dq;

	  // propagate orientation
	  const double theta_half = omega_S.norm()*dt*0.5;
	  const double sinc_theta_half = sinc_test(theta_half);
	  const double cos_theta_half = cos(theta_half);
	  dq.vec()=sinc_theta_half*0.5*dt*omega_S;
	  dq.w()=cos_theta_half;
	  q = q * dq;

	  // propagate speed
	  v+=dt*a_W;

	  // propagate position
	  r+=dt*v;

	  // T_WS
	  T_WS = okvis::kinematics::Transformation(r,q);

	  // speedAndBias - v only, obviously, since this is the Ground Truth
	  speedAndBias.head<3>()=v;

	  // generate measurements
	  Eigen::Vector3d gyr = omega_S + imuParameters.sigma_g_c/sqrt(dt)
			  *Eigen::Vector3d::Random();
	  Eigen::Vector3d acc = T_WS.inverse().C()*(a_W+Eigen::Vector3d(0,0,imuParameters.g)) + imuParameters.sigma_a_c/sqrt(dt)
				*Eigen::Vector3d::Random();
	  imuMeasurements.push_back(okvis::ImuMeasurement(okvis::Time(time),okvis::ImuSensorReadings(gyr,acc)));
	}

	// create the pose parameter blocks
	okvis::kinematics::Transformation T_disturb;
	T_disturb.setRandom(1,0.02);
	okvis::kinematics::Transformation T_WS_1_disturbed=T_WS_1*T_disturb; //
	okvis::ceres::PoseParameterBlock poseParameterBlock_0(T_WS_0,0,t_0); // ground truth
	okvis::ceres::PoseParameterBlock poseParameterBlock_1(T_WS_1_disturbed,2,t_1); // disturbed...
	problem.AddParameterBlock(poseParameterBlock_0.parameters(),okvis::ceres::PoseParameterBlock::Dimension);
	problem.AddParameterBlock(poseParameterBlock_1.parameters(),okvis::ceres::PoseParameterBlock::Dimension);
	//problem.SetParameterBlockConstant(poseParameterBlock_0.parameters());

	// create the speed and bias
	okvis::ceres::SpeedAndBiasParameterBlock speedAndBiasParameterBlock_0(speedAndBias_0,1,t_0);
	okvis::ceres::SpeedAndBiasParameterBlock speedAndBiasParameterBlock_1(speedAndBias_1,3,t_1);
	problem.AddParameterBlock(speedAndBiasParameterBlock_0.parameters(),okvis::ceres::SpeedAndBiasParameterBlock::Dimension);
	problem.AddParameterBlock(speedAndBiasParameterBlock_1.parameters(),okvis::ceres::SpeedAndBiasParameterBlock::Dimension);

	// let's use our own local quaternion perturbation
	std::cout<<"setting local parameterization for pose... "<<std::flush;
	::ceres::LocalParameterization* poseLocalParameterization2d = new okvis::ceres::PoseLocalParameterization2d;
	::ceres::LocalParameterization* poseLocalParameterization = new okvis::ceres::PoseLocalParameterization;
	problem.SetParameterization(poseParameterBlock_0.parameters(),poseLocalParameterization2d);
	problem.SetParameterization(poseParameterBlock_1.parameters(),poseLocalParameterization);
	std::cout<<" [ OK ] "<<std::endl;

	// create the Imu error term
	okvis::ceres::ImuError* cost_function_imu = new okvis::ceres::ImuError(imuMeasurements, imuParameters,t_0, t_1);
	problem.AddResidualBlock(cost_function_imu, NULL,
		  poseParameterBlock_0.parameters(), speedAndBiasParameterBlock_0.parameters(),
		  poseParameterBlock_1.parameters(), speedAndBiasParameterBlock_1.parameters());

	// let's also add some priors to check this alongside
	::ceres::CostFunction* cost_function_pose = new okvis::ceres::PoseError(T_WS_0, 1e-12, 1e-4); // pose prior...
	problem.AddResidualBlock(cost_function_pose, NULL,poseParameterBlock_0.parameters());
	::ceres::CostFunction* cost_function_speedAndBias = new okvis::ceres::SpeedAndBiasError(speedAndBias_0, 1e-12, 1e-12, 1e-12); // speed and biases prior...
	problem.AddResidualBlock(cost_function_speedAndBias, NULL,speedAndBiasParameterBlock_0.parameters());

	// check Jacobians: only by manual inspection...
	// they verify pretty badly due to the fact that the information matrix is also a function of the states
	double* parameters[4];
	parameters[0]=poseParameterBlock_0.parameters();
	parameters[1]=speedAndBiasParameterBlock_0.parameters();
	parameters[2]=poseParameterBlock_1.parameters();
	parameters[3]=speedAndBiasParameterBlock_1.parameters();
	double* jacobians[4];
	Eigen::Matrix<double,15,7,Eigen::RowMajor> J0;
	Eigen::Matrix<double,15,9,Eigen::RowMajor> J1;
	Eigen::Matrix<double,15,7,Eigen::RowMajor> J2;
	Eigen::Matrix<double,15,9,Eigen::RowMajor> J3;
	jacobians[0]=J0.data();
	jacobians[1]=J1.data();
	jacobians[2]=J2.data();
	jacobians[3]=J3.data();
	double* jacobiansMinimal[4];
	Eigen::Matrix<double,15,6,Eigen::RowMajor> J0min;
	Eigen::Matrix<double,15,9,Eigen::RowMajor> J1min;
	Eigen::Matrix<double,15,6,Eigen::RowMajor> J2min;
	Eigen::Matrix<double,15,9,Eigen::RowMajor> J3min;
	jacobiansMinimal[0]=J0min.data();
	jacobiansMinimal[1]=J1min.data();
	jacobiansMinimal[2]=J2min.data();
	jacobiansMinimal[3]=J3min.data();
	Eigen::Matrix<double,15,1> residuals;
	// evaluate twice to be sure that we will be using the linearisation of the biases (i.e. no preintegrals redone)
	static_cast<okvis::ceres::ImuError*>(cost_function_imu)->EvaluateWithMinimalJacobians(parameters,residuals.data(),jacobians,jacobiansMinimal);
	static_cast<okvis::ceres::ImuError*>(cost_function_imu)->EvaluateWithMinimalJacobians(parameters,residuals.data(),jacobians,jacobiansMinimal);

	// and now num-diff:
	double dx=1e-6;

	Eigen::Matrix<double,15,6> J0_numDiff;
	for(size_t i=0; i<6; ++i){
	  Eigen::Matrix<double,6,1> dp_0;
	  Eigen::Matrix<double,15,1> residuals_p;
	  Eigen::Matrix<double,15,1> residuals_m;
	  dp_0.setZero();
	  dp_0[i]=dx;
	  poseLocalParameterization->Plus(parameters[0],dp_0.data(),parameters[0]);
	  //std::cout<<poseParameterBlock_0.estimate().T()<<std::endl;
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_p.data(),NULL);
	  //std::cout<<residuals_p.transpose()<<std::endl;
	  poseParameterBlock_0.setEstimate(T_WS_0); // reset
	  dp_0[i]=-dx;
	  //std::cout<<residuals.transpose()<<std::endl;
	  poseLocalParameterization->Plus(parameters[0],dp_0.data(),parameters[0]);
	  //std::cout<<poseParameterBlock_0.estimate().T()<<std::endl;
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_m.data(),NULL);
	  //std::cout<<residuals_m.transpose()<<std::endl;
	  poseParameterBlock_0.setEstimate(T_WS_0); // reset
	  J0_numDiff.col(i)=(residuals_p-residuals_m)*(1.0/(2*dx));
	}
	OKVIS_ASSERT_TRUE(Exception,(J0min-J0_numDiff).norm()<jacobianTolerance,
	                  "minimal Jacobian 0 = \n"<<J0min<<std::endl<<
	                  "numDiff minimal Jacobian 0 = \n"<<J0_numDiff);
	//std::cout << "minimal Jacobian 0 = \n"<<J0min<<std::endl;
	//std::cout << "numDiff minimal Jacobian 0 = \n"<<J0_numDiff<<std::endl;
	Eigen::Matrix<double,7,6,Eigen::RowMajor> Jplus;
	poseLocalParameterization->ComputeJacobian(parameters[0],Jplus.data());
	//std::cout << "Jacobian 0 times Plus Jacobian = \n"<<J0*Jplus<<std::endl;

	Eigen::Matrix<double,15,6> J2_numDiff;
	for(size_t i=0; i<6; ++i){
	  Eigen::Matrix<double,6,1> dp_1;
	  Eigen::Matrix<double,15,1> residuals_p;
	  Eigen::Matrix<double,15,1> residuals_m;
	  dp_1.setZero();
	  dp_1[i]=dx;
	  poseLocalParameterization->Plus(parameters[2],dp_1.data(),parameters[2]);
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_p.data(),NULL);
	  poseParameterBlock_1.setEstimate(T_WS_1_disturbed); // reset
	  dp_1[i]=-dx;
	  poseLocalParameterization->Plus(parameters[2],dp_1.data(),parameters[2]);
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_m.data(),NULL);
	  poseParameterBlock_1.setEstimate(T_WS_1_disturbed); // reset
	  J2_numDiff.col(i)=(residuals_p-residuals_m)*(1.0/(2*dx));
	}
	OKVIS_ASSERT_TRUE(Exception,(J2min-J2_numDiff).norm()<jacobianTolerance,
	                    "minimal Jacobian 2 = \n"<<J2min<<std::endl<<
	                    "numDiff minimal Jacobian 2 = \n"<<J2_numDiff);
	poseLocalParameterization->ComputeJacobian(parameters[2],Jplus.data());
	//std::cout << "Jacobian 2 times Plus Jacobian = \n"<<J2*Jplus<<std::endl;

	Eigen::Matrix<double,15,9> J1_numDiff;
	for(size_t i=0; i<9; ++i){
	  Eigen::Matrix<double,9,1> ds_0;
	  Eigen::Matrix<double,15,1> residuals_p;
	  Eigen::Matrix<double,15,1> residuals_m;
	  ds_0.setZero();
	  ds_0[i]=dx;
	  Eigen::Matrix<double,9,1> plussed=speedAndBias_0+ds_0;
	  speedAndBiasParameterBlock_0.setEstimate(plussed);
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_p.data(),NULL);
	  ds_0[i]=-dx;
	  plussed=speedAndBias_0+ds_0;
	  speedAndBiasParameterBlock_0.setEstimate(plussed);
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_m.data(),NULL);
	  speedAndBiasParameterBlock_0.setEstimate(speedAndBias_0); // reset
	  J1_numDiff.col(i)=(residuals_p-residuals_m)*(1.0/(2*dx));
	}
	OKVIS_ASSERT_TRUE(Exception,(J1min-J1_numDiff).norm()<jacobianTolerance,
	                      "minimal Jacobian 1 = \n"<<J1min<<std::endl<<
	                      "numDiff minimal Jacobian 1 = \n"<<J1_numDiff);
	//std::cout << "minimal Jacobian 1 = \n"<<J1min<<std::endl;
	//std::cout << "numDiff minimal Jacobian 1 = \n"<<J1_numDiff<<std::endl;

	Eigen::Matrix<double,15,9> J3_numDiff;
	for(size_t i=0; i<9; ++i){
	  Eigen::Matrix<double,9,1> ds_1;
	  Eigen::Matrix<double,15,1> residuals_p;
	  Eigen::Matrix<double,15,1> residuals_m;
	  ds_1.setZero();
	  ds_1[i]=dx;
	  Eigen::Matrix<double,9,1> plussed=speedAndBias_1+ds_1;
	  speedAndBiasParameterBlock_1.setEstimate(plussed);
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_p.data(),NULL);
	  ds_1[i]=-dx;
	  plussed=speedAndBias_1+ds_1;
	  speedAndBiasParameterBlock_1.setEstimate(plussed);
	  static_cast<okvis::ceres::ImuError*>(cost_function_imu)->Evaluate(parameters,residuals_m.data(),NULL);
	  speedAndBiasParameterBlock_1.setEstimate(speedAndBias_0); // reset
	  J3_numDiff.col(i)=(residuals_p-residuals_m)*(1.0/(2*dx));
	}
	OKVIS_ASSERT_TRUE(Exception,(J3min-J3_numDiff).norm()<jacobianTolerance,
	                        "minimal Jacobian 1 = \n"<<J3min<<std::endl<<
	                        "numDiff minimal Jacobian 1 = \n"<<J3_numDiff);
	//std::cout << "minimal Jacobian 3 = \n"<<J3min<<std::endl;
	//std::cout << "numDiff minimal Jacobian 3 = \n"<<J3_numDiff<<std::endl;

	// Run the solver!
	std::cout<<"run the solver... "<<std::endl;
	::ceres::Solver::Options options;
	//options.check_gradients=true;
	//options.numeric_derivative_relative_step_size = 1e-6;
	//options.gradient_check_relative_precision=1e-2;
	options.minimizer_progress_to_stdout = false;
	::FLAGS_stderrthreshold=google::WARNING; // enable console warnings (Jacobian verification)
	::ceres::Solver::Summary summary;
	::ceres::Solve(options, &problem, &summary);

	// print some infos about the optimization
	//std::cout << summary.FullReport() << "\n";
	std::cout << "initial T_WS_1 : " << T_WS_1_disturbed.T() << "\n"
			<< "optimized T_WS_1 : " << poseParameterBlock_1.estimate().T() << "\n"
			<< "correct T_WS_1 : " << T_WS_1.T() << "\n";

	// make sure it converged
	OKVIS_ASSERT_TRUE(Exception,summary.final_cost<1e-2,"cost not reducible");
	OKVIS_ASSERT_TRUE(Exception,2*(T_WS_1.q()*poseParameterBlock_1.estimate().q().inverse()).vec().norm()<1e-2,"quaternions not close enough");
	OKVIS_ASSERT_TRUE(Exception,(T_WS_1.r()-poseParameterBlock_1.estimate().r()).norm()<0.04,"translation not close enough");
}



