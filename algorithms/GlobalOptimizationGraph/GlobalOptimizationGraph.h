#include "G2OTypes.h"
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "GPSExpand.h"
#include <cmath>
using namespace std;
using namespace ygz;
//struct State//the prev-state of drone.
//{
//    
//};
typedef VertexPR State;

const int GPS_INIT_BUFFER_SIZE = 100;
const int GPS_AVAIL_MINIMUM = 50;
class GlobalOptimizationGraph
{
public:
    GlobalOptimizationGraph(const std::string& config_file_path);
    
    bool checkAHRSValid();
    bool checkSLAMValid();
    bool checkQRValid();
    bool checkGPSValid(const sensor_msgs::NavSatFix& gps);
    
    
    bool init_gps()//init longitude,latitude,altitude.
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
    
    bool inputGPS(const sensor_msgs::NavSatFix& gps)
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
    bool isWorkingWithGPS()
    {
      return(this->allow_gps_usage&& this->gps_init_success);
    }
    
    
    
    void addBlockAHRS();
    void addBlockSLAM();
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();
    void addGPS();
    void doOptimization();


private:
    std::vector<State> historyStates;
    State currentState;
    G2OOptimizationGraph graph;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg *solver;
    
    bool allow_gps_usage = true;//can be configured by config.yaml
    bool gps_init_success = false;
    double gps_init_longitude,gps_init_latitude,gps_init_altitude;
    
    std::vector<sensor_msgs::NavSatFix> gps_info_buffer;

    int EdgeID = 0;
    int VertexID = 0;
};
GlobalOptimizationGraph::GlobalOptimizationGraph()
{

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    currentState.setEstimate(currentState);
    currentState.setId(0);//vertex 0 in optimization graph.
    optimizer.addVertex(&currentState);
}/*
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









