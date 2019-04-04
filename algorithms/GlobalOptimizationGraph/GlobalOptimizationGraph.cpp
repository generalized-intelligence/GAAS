#include "GlobalOptimizationGraph.h"
#include <ros/ros.h>
#include "CheckValid.h"
GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{
    cv::FileStorage fSettings(string(argv[1]),cv::FileStorage::READ);
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
    Eigen::Quaterniond q_() = q;
    Matrix3d R_init = q_.toRotationMatrix();
    //TODO:set R_init into graph state.
    this->currentState.R = R_init;
}
bool GlobalOptimizationGraph::init_SLAM(const geometry_msgs::PoseStamped& slam_msg)
{
    //match rotation matrix and translation vector to E,0.
    auto q = slam_msg.pose.pose.orientation;//TODO
    auto t_slam = slam_msg.pose.pose.position;
    SLAM_to_UAV_coordinate_transfer.R = q.toRotationMatrix().inverse() *this->R_init;
    SLAM_to_UAV_coordinate_transfer.t = -1 * SLAM_to_UAV_coordinate_transfer.R * t_slam;
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
	cout<<"GPS Initiated at LONGITUDE:"<<avg_lon<<",LATITUDE:"<<avg_lat<<",ALTITUDE:"<<avg_alt<<".VARIANCE:"<<vari_lon<<", "<<vari_lat<<", "<<vari_alt"."<<endl;
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
    if(lon_variance_m> (*(this->pSettings))['GPS_INIT_VARIANCE_THRESHOLD_m'] || lat_variance_m > (*(this->pSettings))['GPS_INIT_VARIANCE_THRESHOLD_m']
        || vari_alt>(*(this->pSettings))['GPS_INIT_ALT_VARIANCE_THRESHOLD_m'])
    {
        cout<<"WARNING:GPS init failed.VARIANCE out of threshold."<<endl;
        cout<<"THRESHOLD(m):"<<(*(this->pSettings))['GPS_INIT_VARIANCE_THRESHOLD_m']<<endl;
        return false;
    }

    this->gps_init_longitude = avg_lon;
    this->gps_init_latitude = avg_lat;
    this->gps_init_altitude = avg_alt;

    this->gps_init_lon_variance = vari_lon;
    this->gps_init_lat_variance = vari_lat;
    this->gps_init_alt_variance = vari_alt;

	return true;
}
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
    pEdgeAttitude->setMeasurement();
    pEdgeAttitude->setInformation();
    pEdgeAttitude->setLevel(!checkAHRSValid());
    pEdgeAttitude->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));

    
    this->optimizer.addEdge(pEdgeAttitude);
}
void GlobalOptimizationGraph::addBlockGPS(const nav_msgs::NavSatFix& GPS_msg)
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
  pEdgePRGPS->setMeasurement(delta_lon*1000*GPS_coord.vari_km_per_lon_deg(),
                                delta_lat*1000*GPS_coord.vari_km_per_lat_deg(),
                                delta_alt);

  double info_lon,info_lat,info_alt;
  info_lon = min((1.0/this->gps_init_lon_variance),1.0/(*(this->pSettings))["GPS_MIN_VARIANCE_LONLAT_m"]);
  info_lat = min((1.0/this->gps_init_lat_variance),1.0/(*(this->pSettings))["GPS_MIN_VARIANCE_LONLAT_m"]);
  info_alt = min((1.0/this->gps_init_alt_variance),1.0/(*(this->pSettings))["GPS_MIN_VARIANCE_ALT_m"]);


  pEdgePRGPS->setInformation(...);//the inverse mat of covariance.

  pEdgePRGPS->setLevel(!checkGPSValid());
  this->optimizer.addEdge(pEdgePRGPS);
}

bool GlobalOptimizationGraph::estimateCurrentSpeed()
{
  //step<1> check status change log;
  //step<2> if status not changed;calc current speed.
  //step<3> set into vector.
}


GlobalOptimizationGraph::addBlockSLAM()
{
    pEdgeSlam = new EdgeAttitude();
    shared_ptr<g2o::BaseEdge> ptr_slam(pEdgeSlam);
    pEdgeSlam->setId(this->EdgeVec.size());
    this->EdgeVec.push_back(ptr_slam);
    pEdgeSlam->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(this->pCurrentPR.get()));

    pEdgeSlam->setMeasurement(...);
    pEdgeSlam->setInformation(...);
    optimizer.addEdge(pEdgeSlam);
}
GlobalOptimizationGraph::addBlockQRCode()
{
    pEdgeQRCode = 
}
void addBlockSceneRetriever_StrongCoupling(); //Solve se(3) from multiple points PRXYZ;
void addBlockSceneRetriever_WeakCoupling();//just do square dist calc.

GlobalOptimizationGraph::addBlockSceneRetriever()
{
    //step<1>.add vertex PR for scene.
    //set infomation matrix that optimize Rotation and altitude.Do not change longitude and latitude.
    //pBlockSceneRetriever = 
}
GlobalOptimizationGraph::addBlockFCAttitude()
{
    //just call addBlockAHRS.
    this->addBlockAHRS();
}
GlobalOptimizationGraph::addBlockAHRS()
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

}
GlobalOptimizationGraph::doOptimization()
{
    this->optimizer.initializeOptimization();
    this->optimizer.optimize(10);
    this->historyStates.push_back(currentState);
    this->historyStates.reserve();///...
}






/*
int main(int argc,char** argv)
{
    ros::init(argc,argv,"GlobalOptimizationGraph_Node");
    //ros::Subscriber()
    return 0;
}*/
