#include "GlobalOptimizationGraph.h"
#include <ros/ros.h>
#include "CheckValid.h"


/*
bool GlobalOptimizationGraph::SpeedInitialization() {
        // 速度 初始化 需要由后端提供关键帧（不需要地图）
        std::deque<shared_ptr<Frame>> vpKFs = mpBackEnd->GetAllKF();
        int N = vpKFs.size();
        if (N < setting::minInitKFs)    // 初始化需要若干个关键帧
            return false;

        // Note 1.
        // Input : N (N>=4) KeyFrame/Frame poses of stereo vslam
        //         Assume IMU bias are identical for N KeyFrame/Frame
        // Compute :
        //          bg: gyroscope bias
        //          ba: accelerometer bias
        //          gv: gravity in vslam frame
        //          Vwbi: velocity of N KeyFrame/Frame
        // (Absolute scale is available for stereo vslam)

        // Note 2.
        // Convention by wangjing:
        // W: world coordinate frame (z-axis aligned with gravity, gW=[0;0;~9.8])
        // B: body coordinate frame (IMU)
        // V: camera frame of first camera (vslam's coordinate frame)
        // TWB/T : 6dof pose of frame, TWB = [RWB, PWB; 0, 1], XW = RWB*XW + PWB
        // RWB/R : 3dof rotation of frame's body(IMU)
        // PWB/P : 3dof translation of frame's body(IMU) in world
        // XW/XB : 3dof point coordinate in world/body

        // Step0. get all keyframes in map
        //        reset v/bg/ba to 0
        //        re-compute pre-integration

        Vector3d v3zero = Vector3d::Zero();
        for (auto pKF: vpKFs) {
            pKF->SetBiasG(v3zero);
            pKF->SetBiasA(v3zero);
        }
        for (int i = 1; i < N; i++) {
            vpKFs[i]->ComputeIMUPreInt();
        }

        // Step1. gyroscope bias estimation
        //        update bg and re-compute pre-integration
        // 第一步，估计陀螺偏置
        Vector3d bgest = IMUInitEstBg(vpKFs);
        // 重新计算预积分器
        for (auto pKF: vpKFs) {
            pKF->SetBiasG(bgest);
        }
        for (int i = 1; i < N; i++) {
            vpKFs[i]->ComputeIMUPreInt();
        }

        // Step2. accelerometer bias and gravity estimation (gv = Rvw*gw)
        // let's first assume ba is given by prior and solve the gw
        // Step 2.1 gravity estimation

        // Solve C*x=D for x=[gw] (3+3)x1 vector
        // \see section IV in "Visual Inertial Monocular SLAM with Map Reuse"
        Vector3d baPrior = setting::biasAccePrior;

        MatrixXd C(3 * (N - 2), 3);
        C.setZero();

        VectorXd D(3 * (N - 2));
        D.setZero();

        Matrix3d I3 = Matrix3d::Identity();
        for (int i = 0; i < N - 2; i++) {

            // 三个帧才能建立加速度约束
            shared_ptr<Frame> pKF1 = vpKFs[i];
            shared_ptr<Frame> pKF2 = vpKFs[i + 1];
            shared_ptr<Frame> pKF3 = vpKFs[i + 2];

            // Poses
            Matrix3d R1 = pKF1->mRwb.matrix();
            Matrix3d R2 = pKF2->mRwb.matrix();
            Vector3d p1 = pKF1->mTwb;
            Vector3d p2 = pKF2->mTwb;
            Vector3d p3 = pKF3->mTwb;

            // Delta time between frames
            double dt12 = pKF2->mIMUPreInt.getDeltaTime();
            double dt23 = pKF3->mIMUPreInt.getDeltaTime();
            // Pre-integrated measurements
            Vector3d dp12 = pKF2->mIMUPreInt.getDeltaP();
            Vector3d dv12 = pKF2->mIMUPreInt.getDeltaV();
            Vector3d dp23 = pKF3->mIMUPreInt.getDeltaP();

            Matrix3d Jpba12 = pKF2->mIMUPreInt.getJPBiasa();
            Matrix3d Jvba12 = pKF2->mIMUPreInt.getJVBiasa();
            Matrix3d Jpba23 = pKF3->mIMUPreInt.getJPBiasa();

            // 谜之计算
            Matrix3d lambda = 0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * I3;
            Vector3d phi = R2 * Jpba23 * baPrior * dt12 -
                           R1 * Jpba12 * baPrior * dt23 +
                           R1 * Jvba12 * baPrior * dt12 * dt23;
            Vector3d gamma = p3 * dt12 + p1 * dt23 + R1 * dp12 * dt23 - p2 * (dt12 + dt23)
                             - R2 * dp23 * dt12 - R1 * dv12 * dt12 * dt23;

            C.block<3, 3>(3 * i, 0) = lambda;
            D.segment<3>(3 * i) = gamma - phi;

        }

        // Use svd to compute C*x=D, x=[ba] 6x1 vector
        // Solve Ax = b where x is ba
        JacobiSVD<MatrixXd> svd2(C, ComputeThinU | ComputeThinV);
        VectorXd y = svd2.solve(D);
        Vector3d gpre = y.head(3);
        // normalize g
        Vector3d g0 = gpre / gpre.norm() * setting::gravity;

        // Step2.2
        // estimate the bias from g
        MatrixXd A(3 * (N - 2), 3);
        A.setZero();
        VectorXd B(3 * (N - 2));
        B.setZero();

        for (int i = 0; i < N - 2; i++) {

            // 三个帧才能建立加速度约束
            shared_ptr<Frame> pKF1 = vpKFs[i];
            shared_ptr<Frame> pKF2 = vpKFs[i + 1];
            shared_ptr<Frame> pKF3 = vpKFs[i + 2];

            // Poses
            Matrix3d R1 = pKF1->mRwb.matrix();
            Matrix3d R2 = pKF2->mRwb.matrix();
            Vector3d p1 = pKF1->mTwb;
            Vector3d p2 = pKF2->mTwb;
            Vector3d p3 = pKF3->mTwb;

            // Delta time between frames
            double dt12 = pKF2->mIMUPreInt.getDeltaTime();
            double dt23 = pKF3->mIMUPreInt.getDeltaTime();
            // Pre-integrated measurements
            Vector3d dp12 = pKF2->mIMUPreInt.getDeltaP();
            Vector3d dv12 = pKF2->mIMUPreInt.getDeltaV();
            Vector3d dp23 = pKF3->mIMUPreInt.getDeltaP();

            Matrix3d Jpba12 = pKF2->mIMUPreInt.getJPBiasa();
            Matrix3d Jvba12 = pKF2->mIMUPreInt.getJVBiasa();
            Matrix3d Jpba23 = pKF3->mIMUPreInt.getJPBiasa();

            // 谜之计算
            Vector3d lambda = 0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * I3 * g0;
            Matrix3d phi = R2 * Jpba23 * dt12 -
                           R1 * Jpba12 * dt23 +
                           R1 * Jvba12 * dt12 * dt23;
            Vector3d gamma = p3 * dt12 + p1 * dt23 + R1 * dp12 * dt23 - p2 * (dt12 + dt23)
                             - R2 * dp23 * dt12 - R1 * dv12 * dt12 * dt23;

            A.block<3, 3>(3 * i, 0) = phi;
            B.segment<3>(3 * i) = gamma - lambda;
        }

        JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
        VectorXd y2 = svd.solve(B);
        Vector3d baest = y2;

        // update ba and re-compute pre-integration
        for (auto pkf : vpKFs) {
            pkf->SetBiasA(baest);
        }
        for (int i = 1; i < N; i++) {
            vpKFs[i]->ComputeIMUPreInt();
        }

        // Step3. velocity estimation
        for (int i = 0; i < N; i++) {
            auto pKF = vpKFs[i];
            if (i != N - 1) {
                // not last KeyFrame, R1*dp12 = p2 - p1 -v1*dt12 - 0.5*gw*dt12*dt12
                //  ==>> v1 = 1/dt12 * (p2 - p1 - 0.5*gw*dt12*dt12 - R1*dp12)

                auto pKF2 = vpKFs[i + 1];
                const Vector3d p2 = pKF2->mTwb;
                const Vector3d p1 = pKF->mTwb;
                const Matrix3d R1 = pKF->mRwb.matrix();
                const double dt12 = pKF2->mIMUPreInt.getDeltaTime();
                const Vector3d dp12 = pKF2->mIMUPreInt.getDeltaP();

                Vector3d v1 = (p2 - p1 - 0.5 * g0 * dt12 * dt12 - R1 * dp12) / dt12;
                pKF->SetSpeed(v1);
            } else {
                // last KeyFrame, R0*dv01 = v1 - v0 - gw*dt01 ==>> v1 = v0 + gw*dt01 + R0*dv01
                auto pKF0 = vpKFs[i - 1];
                const Matrix3d R0 = pKF0->mRwb.matrix();
                const Vector3d v0 = pKF0->mSpeedAndBias.segment<3>(0);
                const double dt01 = pKF->mIMUPreInt.getDeltaTime();
                const Vector3d dv01 = pKF->mIMUPreInt.getDeltaV();

                Vector3d v1 = v0 + g0 * dt01 + R0 * dv01;
                pKF->SetSpeed(v1);
            }
        }

        double gprenorm = gpre.norm();
        // double baestdif = (baest0 - baest).norm();

        LOG(INFO) << "Estimated gravity before: " << gpre.transpose() << ", |gw| = " << gprenorm << endl;
        LOG(INFO) << "Estimated acc bias after: " << baest.transpose() << endl;
        LOG(INFO) << "Estimated gyr bias: " << bgest.transpose() << endl;

        bool initflag = false;
        if (gprenorm > 9.7 && gprenorm < 9.9 && 
            baest.norm() < 1) {
            LOG(INFO) << "IMU init ok!" << endl;
            initflag = true;
        } else {
            // goodcnt = 0;
        }

        // align 'world frame' to gravity vector, making mgWorld = [0,0,9.8]
        if (initflag) {
            mgWorld = g0;
        }
        return initflag;
    }
*/

 /*   
bool GlobalOptimizationGraph::inputGPS(const sensor_msgs::NavSatFix& gps)
{
    if(this->allow_gps_usage==false)
    {
		cout<<"[WARNING] GPS Usage refused in config file."<<endl;
		return false;
    }
    if(this->gps_init_success == false)
    {
        if(this->gps_info_buffer.size()<GPS_INIT_BUFFER_SIZE)
		{
	    	this->gps_info_buffer.push_back(gps);
		}
		else
		{
		    this->gps_init_success = this->init_gps();
		    if(this->gps_init_success == false)
		    {
				this->gps_info_buffer.empty();
		    }
		}
		return this->gps_init_success;
    }
    else
    {
		bool retval = checkGPSValid(gps);
		if (retval)
		{
	    	this->addGPS(gps);
		}
		return retval;
    }
}*/

/*void GlobalOptimizationGraph::addBlockFCAttitude()
{
    //just call addBlockAHRS.
    this->addBlockAHRS();
}*/

/*
void GlobalOptimizationGraph::addBlockAHRS(const nav_msgs::Odometry& AHRS_msg)
{
    pEdgeAHRS = new EdgeAttitude();
    shared_ptr<g2o::BaseEdge> ptr_ahrs(pEdgeAHRS);
    pEdgeAHRS->setId(this->EdgeVec.size());
    this->EdgeVec.push_back(ptr_ahrs);
    pEdgeAHRS->setMeasurement(...);
    pEdgeAHRS->setInformation(...);
    optimizer.addEdge(pEdgeAHRS)
    if(check_avail())
    {
        ;
    }
    else
    {
        pEdgeAHRS->setLevel(1);
    }

}*/
