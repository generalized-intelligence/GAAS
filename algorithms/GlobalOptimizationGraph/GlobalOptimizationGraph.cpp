#include "GlobalOptimizationGraph.h"
#include <ros/ros.h>
GlobalOptimizationGraph::GlobalOptimizationGraph(int argc,char** argv)
{
    cv::FileStorage fSettings(string(argv[1]),cv::FileStorage::READ);
    this->GPS_AVAIL_MINIMUM = fSettings["GPS_AVAIL_MINIMUM"];
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    currentState.setEstimate(currentState);
    currentState.setId(0);//vertex 0 in optimization graph.
    optimizer.addVertex(&currentState);
}
bool GlobalOptimizationGraph::init_AHRS(const nav_msgs::Odometry& AHRS_msg)
{
    auto q = AHRS_msg.pose.pose.orientation;
    
    //Matrix3d m = q.....
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
	cout<<"GPS Initiated at LONGITUDE:"<<avg_lon<<",LATITUDE:"<<avg_lat<<",ALTITUDE:"<<avg_alt<<".VARIANCE:"<<vari_lon<<", "<<vari_lat<<", "<<vari_alt"."<<endl;
	cout<<"Available count:"<<count<<"."<<endl;
	
	//expand at avg lon,lat.
	GPSExpand GE;
	GE.expandAt(avg_lon,avg_lat,avg_alt);
	cout<<"X variance:"<<GE.vari_km_per_lon_deg()*vari_lon*1000<<"m;Y Variance:"<<GE.vari_km_per_lat_deg()*vari_lat*1000<<"m."<<endl;
	return true;
}
    
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
}
void GlobalOptimizationGraph::addBlockAHRS()
{
    EdgeAttitude* pEdgeAttitude = new EdgeAttitude();
    pEdgeAttitude->setMeasurement();
    pEdgeAttitude->setInformation();
    pEdgeAttitude->setLevel(!checkAHRSValid());//enable AHRS?
    pEdgeAttitude->setVertex();
    
    this->optimizer.addEdge(pEdgeAttitude);
}
void GlobalOptimizationGraph::addGPS()
{
  if(this->allow_gps_usage == false)
  {
    cout<<"[WARNING] Unable to add GPS edge.GPS usage is forbidden in config file."<<endl;
    return;
  }
  if(!(this->status&this->STATUS_WITH_GPS_NO_SCENE))
  {
    //match with new coordinate
    this->GPS_coord.init_at(xxx_msg);
  }
  EdgePRGPS* pEdgePRGPS = new EdgePRGPS();
  pEdgePRGPS->setMeasurement(xxx_msg-this->GPS_coord.xxx);
  pEdgePRGPS->setInformation();
  pEdgePRGPS->setLevel(!checkGPSValid());
  this->optimizer.addEdge(pEdgePRGPS);
}

bool GlobalOptimizationGraph::estimateCurrentSpeed()
{
  //step<1> check status change log;
  //step<2> if status not changed;calc current speed.
  //step<3> set into vector.
}

/*
GlobalOptimizationGraph::addBlockSLAM()
{
    pEdgeSlam = new EdgeAttitude();
    pEdgeSlam->setId(this->EdgeID);
    this->EdgeID++;
    pEdgeSlam->setMeasurement(...);
    pEdgeSlam->setInformation();
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
    pBlockSceneRetriever = 
}
GlobalOptimizationGraph::addBlockFCAttitude()
{
    pEdgeAttitude = ...
}
GlobalOptimizationGraph::addBlockAHRS()
{
    pEdgeAHRS = ....
    if(check_avail())
    {
        ...
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
*/




void subscribeGPSCallback();
/*
int main(int argc,char** argv)
{
    ros::init(argc,argv,"GlobalOptimizationGraph_Node");
    //ros::Subscriber()
    return 0;
}*/
