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
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include "glog/logging.h"
#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/HomogeneousPointError.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, ReprojectionError){
	// initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

	// Build the problem.
	::ceres::Problem problem;

	// set up a random geometry
	std::cout<<"set up a random geometry... "<<std::flush;
	okvis::kinematics::Transformation T_WS; // world to sensor
	T_WS.setRandom(10.0,M_PI);
	okvis::kinematics::Transformation T_disturb;
	T_disturb.setRandom(1,0.01);
	okvis::kinematics::Transformation T_WS_init=T_WS*T_disturb; // world to sensor
	okvis::kinematics::Transformation T_SC; // sensor to camera
	T_SC.setRandom(0.2,M_PI);
	okvis::ceres::PoseParameterBlock poseParameterBlock(T_WS_init,1,okvis::Time(0));
	okvis::ceres::PoseParameterBlock extrinsicsParameterBlock(T_SC,2,okvis::Time(0));
	problem.AddParameterBlock(poseParameterBlock.parameters(),okvis::ceres::PoseParameterBlock::Dimension);
	problem.AddParameterBlock(extrinsicsParameterBlock.parameters(),okvis::ceres::PoseParameterBlock::Dimension);
	problem.SetParameterBlockVariable(poseParameterBlock.parameters()); // optimize this...
	problem.SetParameterBlockConstant(extrinsicsParameterBlock.parameters()); // do not optimize this...
	std::cout<<" [ OK ] "<<std::endl;

	// set up a random camera geometry
        std::cout << "set up a random camera geometry... " << std::flush;
        typedef okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion> DistortedPinholeCameraGeometry;
        std::shared_ptr<const DistortedPinholeCameraGeometry> cameraGeometry =
        std::static_pointer_cast<const DistortedPinholeCameraGeometry>(DistortedPinholeCameraGeometry::createTestObject());
        std::cout << " [ OK ] " << std::endl;

	// let's use our own local quaternion perturbation
	std::cout<<"setting local parameterization for pose... "<<std::flush;
	::ceres::LocalParameterization* poseLocalParameterization = new okvis::ceres::PoseLocalParameterization;

	problem.SetParameterization(poseParameterBlock.parameters(),poseLocalParameterization);
	problem.SetParameterization(extrinsicsParameterBlock.parameters(),poseLocalParameterization);
	std::cout<<" [ OK ] "<<std::endl;

	// and the parameterization for points:
	::ceres::LocalParameterization* homogeneousPointLocalParameterization =
		  new okvis::ceres::HomogeneousPointLocalParameterization;

	// get some random points and build error terms
	const size_t N=100;
	std::cout<<"create N="<<N<<" visible points and add respective reprojection error terms... "<<std::flush;
	for (size_t i=1; i<100; ++i){

	  Eigen::Vector4d point = cameraGeometry->createRandomVisibleHomogeneousPoint(double(i%10)*3+2.0);
	  okvis::ceres::HomogeneousPointParameterBlock* homogeneousPointParameterBlock_ptr =
			  new okvis::ceres::HomogeneousPointParameterBlock(T_WS*T_SC*point,i+2);
	  problem.AddParameterBlock(homogeneousPointParameterBlock_ptr->parameters(),okvis::ceres::HomogeneousPointParameterBlock::Dimension);
	  problem.SetParameterBlockConstant(homogeneousPointParameterBlock_ptr->parameters());

	  // get a randomized projection
	  Eigen::Vector2d kp;
	  cameraGeometry->projectHomogeneous(point,&kp);
	  kp += Eigen::Vector2d::Random();

	  // Set up the only cost function (also known as residual).
	  Eigen::Matrix2d information=Eigen::Matrix2d::Identity();
	  ::ceres::CostFunction* cost_function = new okvis::ceres::ReprojectionError<DistortedPinholeCameraGeometry>(
			  cameraGeometry,1, kp,information);
	  problem.AddResidualBlock(cost_function, NULL, poseParameterBlock.parameters(), homogeneousPointParameterBlock_ptr->parameters(), extrinsicsParameterBlock.parameters());

	  // set the parameterization
	  problem.SetParameterization(homogeneousPointParameterBlock_ptr->parameters(),homogeneousPointLocalParameterization);
	}
	std::cout<<" [ OK ] "<<std::endl;

	// Run the solver!
	std::cout<<"run the solver... "<<std::endl;
	::ceres::Solver::Options options;
	//options.check_gradients=true;
	//options.numeric_derivative_relative_step_size = 1e-6;
	//options.gradient_check_relative_precision=1e-2;
	options.minimizer_progress_to_stdout = false;
	::FLAGS_stderrthreshold=google::WARNING; // enable console warnings (Jacobian verification)
	::ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);

	// verify there are no errors in the Jacobians
	OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

	// print some infos about the optimization
	//std::cout << summary.BriefReport() << "\n";
	std::cout << "initial T_WS : " << T_WS_init.T() << "\n"
			<< "optimized T_WS : " << poseParameterBlock.estimate().T() << "\n"
			<< "correct T_WS : " << T_WS.T() << "\n";

	// make sure it converged
	OKVIS_ASSERT_TRUE(Exception,2*(T_WS.q()*poseParameterBlock.estimate().q().inverse()).vec().norm()<1e-2,"quaternions not close enough");
	OKVIS_ASSERT_TRUE(Exception,(T_WS.r()-poseParameterBlock.estimate().r()).norm()<1e-1,"translation not close enough");
}
