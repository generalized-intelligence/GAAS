#include "GlobalOptimizationGraph.h"
#include <ros/ros.h>
#include "CheckValid.h"


GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{
    cv::FileStorage fSettings;//(string(argv[1]),cv::FileStorage::READ);
    fSettings.open(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    /*currentState.setEstimate(currentState);
    currentState.setId(0);//vertex 0 in optimization graph.
    optimizer.addVertex(&currentState);*/
}
bool GlobalOptimizationGraph::init_AHRS(const nav_msgs::Odometry& AHRS_msg)
{
    auto q = AHRS_msg.pose.pose.orientation;
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    Matrix3d R_init = q_.toRotationMatrix();
    this->ahrs_R_init = R_init;
    //TODO:set R_init into graph state.
    this->currentState.R() = R_init;
    return true;
}
bool GlobalOptimizationGraph::init_SLAM(//const geometry_msgs::PoseStamped& slam_msg
)
{
    //match rotation matrix and translation vector to E,0.
    auto slam_msg = this->pSLAM_Buffer->getLastMessage();
    auto q = slam_msg.pose.orientation;//TODO
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    auto t_slam = slam_msg.pose.position;
    Vector3d t_slam_(t_slam.x,t_slam.y,t_slam.z);

    SLAM_to_UAV_coordinate_transfer.R() = q_.toRotationMatrix().inverse() *this->ahrs_R_init;
    SLAM_to_UAV_coordinate_transfer.t() = -1 * SLAM_to_UAV_coordinate_transfer.R() * t_slam_;
    return true;
}

bool GlobalOptimizationGraph::init_gps()//init longitude,latitude,altitude.
    //TODO:Init a GPS callback buffer block class,inherit callback buffer base,implement init and check avail.
{
    double avg_lon,avg_lat,avg_alt;
	avg_lon = 0;
	avg_lat = 0;
	avg_alt = 0;
	int count = 0;
	//assert gaussian distribution,sigma>3 is rejection region.
	double vari_lon,vari_lat,vari_alt;
	vari_lon=0;
	vari_lat=0;
	vari_alt=0;
    for(auto g:this->gps_info_buffer)
	{
	    if(g.status.status>=g.status.STATUS_FIX) //which means gps available.
	    {
			avg_lon+=g.longitude;
			avg_lat+=g.latitude;
			avg_alt+=g.altitude;
			count++;
	    }
	}
	if(count<GPS_AVAIL_MINIMUM)
	{
	    return false;
	}
	avg_lon = avg_lon/count;
	avg_lat = avg_lat/count;
	avg_alt = avg_alt/count;
	for(auto g:this->gps_info_buffer)
	{
	    if(g.status.status>=g.status.STATUS_FIX)
	    {
	        vari_lon+=pow(g.longitude-avg_lon,2);
			vari_lat+=pow(g.latitude-avg_lat,2);
			vari_alt+=pow(g.altitude-avg_alt,2);
	    }
	}
	vari_lon/=count;
	vari_lat/=count;
	vari_alt/=count;
    vari_lon = sqrt(vari_lon);
    vari_lat = sqrt(vari_lat);
    vari_alt = sqrt(vari_alt);
	cout<<"GPS Initiated at LONGITUDE:"<<avg_lon<<",LATITUDE:"<<avg_lat<<",ALTITUDE:"<<avg_alt<<".VARIANCE:"<<vari_lon<<", "<<vari_lat<<", "<<vari_alt<<"."<<endl;
	cout<<"Available count:"<<count<<"."<<endl;
	
	//expand at avg lon,lat.
	GPSExpand GE;
	GE.expandAt(avg_lon,avg_lat,avg_alt);
    double lon_variance_m,lat_variance_m;
    lon_variance_m = GE.vari_km_per_lon_deg()*vari_lon*1000;
    lat_variance_m = GE.vari_km_per_lat_deg()*vari_lat*1000;

	cout<<"X variance:"<<GE.vari_km_per_lon_deg()*vari_lon*1000<<"m;Y Variance:"
            <<GE.vari_km_per_lat_deg()*vari_lat*1000<<
            "m,ALTITUDE Variance:"<<vari_alt<<"m."<<endl;
    //check variance:
    double gps_init_variance_thres = ((*(this->pSettings))["GPS_INIT_VARIANCE_THRESHOLD_m"]);
    double gps_init_alt_variance_thres =  (*(this->pSettings))["GPS_INIT_ALT_VARIANCE_THRESHOLD_m"];
    if(lon_variance_m> gps_init_variance_thres || lat_variance_m > gps_init_variance_thres
        || vari_alt>gps_init_alt_variance_thres)
    {
        cout<<"WARNING:GPS init failed.VARIANCE out of threshold."<<endl;
        cout<<"THRESHOLD(m):"<<gps_init_variance_thres<<endl;
        return false;
    }

    this->gps_init_longitude = avg_lon;
    this->gps_init_latitude = avg_lat;
    this->gps_init_altitude = avg_alt;

    this->gps_init_lon_variance = vari_lon;
    this->gps_init_lat_variance = vari_lat;
    this->gps_init_alt_variance = vari_alt;

	return true;
}/*
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
void GlobalOptimizationGraph::addBlockAHRS(const nav_msgs::Odometry& AHRS_msg)
{
    EdgeAttitude* pEdgeAttitude = new EdgeAttitude();
    auto q = AHRS_msg.pose.pose.orientation;//TODO
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    //pEdgeAttitude->setMeasurement(q_);//TODO
    Eigen::Matrix<double,3,3> info_mat = Eigen::Matrix<double,3,3>::Identity();
    pEdgeAttitude->setInformation(info_mat);
    pEdgeAttitude->setLevel(!checkAHRSValid());
    pEdgeAttitude->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));

    
    this->optimizer.addEdge(pEdgeAttitude);
}
void GlobalOptimizationGraph::addBlockGPS(const sensor_msgs::NavSatFix& GPS_msg)
{
  if(this->allow_gps_usage == false || this->gps_init_success == false)
  {
    cout<<"[WARNING] Unable to add GPS edge.GPS usage is forbidden in config file."<<endl;
    return;
  }
  /*//state shall be put into state tranfer module.
  if(!(this->status&this->STATUS_WITH_GPS_NO_SCENE))
  {
      //match with new coordinate
      this->GPS_coord.init_at(xxx_msg);
  }*/
  EdgePRGPS* pEdgePRGPS = new EdgePRGPS();
  double delta_lon = GPS_msg.longitude - GPS_coord.getLon();
  double delta_lat = GPS_msg.latitude - GPS_coord.getLat();
  double delta_alt = GPS_msg.altitude - GPS_coord.getAlt();

  //since the quaternion is NED defined,we do not need any rotation here.
  pEdgePRGPS->setMeasurement( Vector3d(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),
                                delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),
                                delta_alt)
                            );

  double info_lon,info_lat,info_alt;
  double gps_min_variance_lonlat_m = (*(this->pSettings))["GPS_MIN_VARIANCE_LONLAT_m"];
  double gps_min_variance_alt_m = (*(this->pSettings))["GPS_MIN_VARIANCE_ALT_m"];
  info_lon = min((1.0/this->gps_init_lon_variance),1.0/gps_min_variance_lonlat_m);
  info_lat = min((1.0/this->gps_init_lat_variance),1.0/gps_min_variance_lonlat_m);
  info_alt = min((1.0/this->gps_init_alt_variance),1.0/gps_min_variance_alt_m);

  Eigen::Matrix<double,3,3> info_mat = Eigen::Matrix<double,3,3>::Identity();
  info_mat(0,0) = info_lon;
  info_mat(1,1) = info_lat;
  info_mat(2,2) = info_alt;
  pEdgePRGPS->setInformation(info_mat);//the inverse mat of covariance.

  //pEdgePRGPS->setLevel(!checkGPSValid());//TODO
  this->optimizer.addEdge(pEdgePRGPS);
}

bool GlobalOptimizationGraph::estimateCurrentSpeed()
{
  //step<1> check status change log;
  //step<2> if status not changed;calc current speed.
  //step<3> set into vector.
}


void GlobalOptimizationGraph::addBlockSLAM(const geometry_msgs::PoseStamped& SLAM_msg)
//if use this api,only reserve the last one.
//void GlobalOptimizationGraph::addBlockSLAM(std::vector<const geometry_msgs::PoseStamped&> SLAM_msg_list)
//for multiple msgs.
{
    //part<1> Rotation.
    auto pEdgeSlam = new EdgeAttitude();
    //shared_ptr<g2o::OptimizableGraph::Edge *> ptr_slam(pEdgeSlam);
    //pEdgeSlam->setId(this->EdgeVec.size());//TODO
    //this->EdgeVec.push_back(ptr_slam);
    pEdgeSlam->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));

    auto q = SLAM_msg.pose.orientation;
    Eigen::Quaterniond q_;
    q_.w() = q.w;
    q_.x() = q.x;
    q_.y() = q.y;
    q_.z() = q.z;
    Eigen::Matrix<double,3,3> info_mat_slam_rotation = Eigen::Matrix<double,3,3>::Identity();
    
    pEdgeSlam->setMeasurement(q_.toRotationMatrix().topLeftCorner(3,3));
    pEdgeSlam->setInformation(info_mat_slam_rotation);
    optimizer.addEdge(pEdgeSlam);
    //part<2> Translation
    //Edge PRV.
    /*//TODO:fill Edge SLAM PRV.
    if(this->historyStates.size()>0)
    {
        shared_ptr<Edge_SLAM_PRV>pEdge_SLAM_PRV(new Edge_SLAM_PRV());
        pEdge_SLAM_PRV->setId(this->EdgeVec.size());
        this->EdgeVec.push_back(pEdge_SLAM_PRV);
        //TODO:fit in type.
        pEdge_SLAM_PRV->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>&(this->historyStates[this->historyStates.size()-1]));//old one.
        pEdge_SLAM_PRV->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));
        pEdge_SLAM_PRV->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex *> (&(this->historySpeed[this->historySpeed.size()-1])));
        pEdge_SLAM_PRV->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex *> (&this->currentSpeed));
        g2o::OptimizableGraph::Vertex::
        //pEdge_SLAM_PRV->setInformation(....);//TODO
    }
    else
    {//just add translation.
        shared_ptr<EdgePRGPS> pEdgePositionSLAM(new EdgePRGPS());
        pEdgePositionSLAM->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get());
        //calc infomation mat from multiple/single slam msg.May be it can be estimated from points num or quality.
        //pEdgePositionSLAM->setInformation(...);TODO
    }*/


    //TODO
}
void GlobalOptimizationGraph::addBlockQRCode()
{
    //pEdgeQRCode = 
}
void addBlockSceneRetriever_StrongCoupling(); //Solve se(3) from multiple points PRXYZ;
void addBlockSceneRetriever_WeakCoupling();//just do square dist calc.

void GlobalOptimizationGraph::addBlockSceneRetriever()
{
    ;//TODO
    //step<1>.add vertex PR for scene.
    //set infomation matrix that optimize Rotation and altitude.Do not change longitude and latitude.
    //pBlockSceneRetriever = 
}
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
void GlobalOptimizationGraph::doOptimization()
{
    this->optimizer.initializeOptimization();
    this->optimizer.optimize(10);
    this->historyStates.push_back(currentState);
    //this->historyStates.reserve();///...
}
