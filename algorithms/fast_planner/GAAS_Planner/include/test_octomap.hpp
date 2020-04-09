#include <iostream>
#include "astar.h"
#include "hybrid_astar.h"
#include <map>
#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "utils/path_visulization.h"
#include <thread>
#include <mutex>
#include <glog/logging.h>
#include <time.h>
#include "utils/path_visulization.h"
#include "utils/scopetimer.h"
#include "plan_enviroment/enviroment.h"
#include "path_optimization/cubic_bspline.h"
#include <IpTNLP.hpp>
#include "path_optimization/bspline_nlp.h"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

class TestOctomap
{
private:
  
  Astar* astar_;
  HybridAstar* hastar_;
  Eigen::Vector3d start_pt_;
  Eigen::Vector3d end_pt_;
  Eigen::Vector3d start_vel_;
  Eigen::Vector3d start_acc_;
  Eigen::Vector3d end_vel_;
  
  PathVisulization *visualization_;
  
  ros::Subscriber octomap_sub;
  
  sensor_msgs::PointCloud out_pointcloud; 
  
  std::vector<Eigen::Vector3d> obs_list_;
  
  
  mutex obstacle_set_mutex;
  
  ros::NodeHandle *nh;
  
  
public:
  
  bool is_set_map;
  
  TestOctomap(ros::NodeHandle* n) : nh(n)
  {
    std::cout<<"Init test."<<std::endl;
    
    
    hastar_ = new HybridAstar();
    hastar_->reset();
    hastar_->setParam();
    
    visualization_ = new PathVisulization(*nh);
    
    is_set_map = false;
    std::cout<<"Init test12."<<std::endl;
    
    //obstacle_set_mutex.lock();
    ros::Rate rate(50.0);
    ros::Time::init();
//     thread t1(&TestOctomap::ros_thread, this);
//     t1.detach();
    
    
    std::cout<<"Init test."<<std::endl;
    
  }
  
  void ros_thread()
  {
     std::cout<<"Ros spawn."<<std::endl;
    octomap_sub = nh->subscribe<sensor_msgs::PointCloud2>
            ("/octomap_point_cloud_centers", 10, &TestOctomap::octomap_update_callback, this);
	ros::spin();    
  }
  
  void getDistance(SdfEnviroment& sf, Eigen::Vector3d& pos)
  {
    sf.getDistance(pos);
  }
  
  
  void search()
  {
    std::cout<<"Start search."<<std::endl;
    start_pt_ = Eigen::Vector3d(0,0,3);
    end_pt_ = Eigen::Vector3d(6,0,3);
    
    clock_t start, end;
    double cost;
    
    start = std::clock();
    int result = astar_->findPath(start_pt_, end_pt_);
    end = std::clock();
    
    cost = end-start;
    std::cout<<"Time start: "<<start<< std::endl;
    std::cout<<"Time end: "<<end<< std::endl;
    std::cout<<"Time cost: "<<cost / 1000.0<< std::endl;
    if (result == 1)
    {
      std::vector< Eigen::Vector3d > path = astar_->getPath();
      std::cout<<"Path: "<<path.size()<<std::endl;
      for (int i=0; i<path.size(); i++)
      {
	std::cout<<path[i](0)<<", "<<path[i](1)<<", "<<path[i](2)<<std::endl;
      }
    }
  }
  
  void searchHybrid(SdfEnviroment& sf)
  {
    hastar_->setMap(sf);
    std::cout<<"Start search."<<std::endl;
    start_pt_ = Eigen::Vector3d(0,0,3);
    end_pt_ = Eigen::Vector3d(7,0,3);
    
    start_vel_ = Eigen::Vector3d(0,0,0);
    end_vel_ = Eigen::Vector3d(0,0,0);
    start_acc_ = Eigen::Vector3d(0,0,0);
    
    clock_t start, end;
    double cost;
    
    start = std::clock();
    ScopeTimer timer("Hybrid ASTAR");
    int result = hastar_->findPath(start_pt_,start_vel_,start_acc_,  end_pt_, end_vel_);
    timer.watch("END Search", false);
    end = std::clock();
    
    cost = end-start;
    std::cout<<"Time start: "<<start<< std::endl;
    std::cout<<"Time end: "<<end<< std::endl;
    std::cout<<"Time cost: "<<cost / 1000<< std::endl;
    if (result == 1)
    {
      std::vector< Eigen::VectorXd > path = hastar_->getPath();
      std::cout<<"Path: "<<path.size()<<std::endl;
      for (int i=0; i<path.size(); i++)
      {
	std::cout<<path[i](0)<<", "<<path[i](1)<<", "<<path[i](2)<<" | "<<
	  path[i](3)<<", "<<path[i](4)<<", "<<path[i](5)<<std::endl;
      }
      
      std::vector<Eigen::Vector3d> h_path = hastar_->getTrajPoints();
      
      for(int i=0; i<30; i++)
      {
	visualization_->drawPath(h_path, 0.1,  Eigen::Vector4d(1, 0, 0, 1));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout<<"Print path..."<<std::endl;
      }
      
    }
    else
    {
      LOG(ERROR)<<"Wrong";
    }
  }
  
  void test_cubic_bspline(SdfEnviroment& sf)
  {
    hastar_->setMap(sf);
    start_pt_ = Eigen::Vector3d(0,0,3);
    end_pt_ = Eigen::Vector3d(7,0,3);
    
    start_vel_ = Eigen::Vector3d(0,0,0);
    end_vel_ = Eigen::Vector3d(0,0,0);
    start_acc_ = Eigen::Vector3d(0,0,0);
    int result = hastar_->findPath(start_pt_,start_vel_,start_acc_,  end_pt_, end_vel_);
    LOG(INFO)<<"Get path.";
    if (result == 1)
    {
      std::vector< Eigen::VectorXd > path = hastar_->getPath();
      std::vector<Eigen::Vector3d> h_path = hastar_->getTrajPoints();
      double ts = 0.5/2.0;
      Eigen::MatrixXd samples = hastar_->getSampleMatrix(ts);//, h_path);
      LOG(INFO)<<"Get samples.";
      CubicBspline bspline;
      bspline.setControlPointFromValuePoint(samples, ts);
      LOG(INFO)<<"Set samples.";
      for(int i=0; i<30; i++)
      {
	visualization_->drawPath(h_path, 0.1,  Eigen::Vector4d(1, 0, 0, 1));
	visualization_->drawBspline(bspline, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1),true, 0.12,Eigen::Vector4d(0, 1, 0, 1));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout<<"Print path..."<<std::endl;
      }
    }
    
  }
  
  void test_optimizer(SdfEnviroment &sf)
  {
    hastar_->setMap(sf);
    start_pt_ = Eigen::Vector3d(0,0,3);
    end_pt_ = Eigen::Vector3d(7,0,3);
    
    start_vel_ = Eigen::Vector3d(0,0,0);
    end_vel_ = Eigen::Vector3d(0,0,0);
    start_acc_ = Eigen::Vector3d(0,0,0);
    int result = hastar_->findPath(start_pt_,start_vel_,start_acc_,  end_pt_, end_vel_);
    LOG(INFO)<<"Get path.";
    if (result == 1)
    {
      std::vector< Eigen::VectorXd > path = hastar_->getPath();
      std::vector<Eigen::Vector3d> h_path = hastar_->getTrajPoints();
      double ts = 0.5/2.0;
      Eigen::MatrixXd samples = hastar_->getSampleMatrix(ts);//, h_path);
      LOG(INFO)<<"Get samples.";
      CubicBspline bspline;
      bspline.setControlPointFromValuePoint(samples, ts);
      Eigen::MatrixXd control_pts = bspline.getControlPoints();
      int n = 3*(control_pts.rows()-3-3);
     
      
      
      Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new BsplineTNLP(n, control_pts, ts, sf);
      Ipopt::SmartPtr<Ipopt::IpoptApplication> app = new Ipopt::IpoptApplication();
      app->Options()->SetStringValue("hessian_approximation", "limited-memory");
      app->Options()->SetIntegerValue("max_iter", 100);
      app->Initialize();
      
      Ipopt::ApplicationReturnStatus status;
      if( status != Solve_Succeeded )
      {
	  std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
	  //return (int) status;
      }
      
      ScopeTimer timer("Hybrid ASTAR");
      status = app->OptimizeTNLP(mynlp);
      
      
      if( status == Solve_Succeeded )
      {
	Ipopt::Index iter_count = app->Statistics()->IterationCount();
	std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;
	Ipopt::Number final_obj = app->Statistics()->FinalObjective();
	std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.'
		  << std::endl;
      }
      
      
      BsplineTNLP* b = (BsplineTNLP*) Ipopt::GetRawPtr(mynlp);
      timer.watch("Optimize", false);
      std::vector<double> best;
      best = b->getResult();
      
      int end_id = control_pts.rows() -3;
      
      
      LOG(INFO)<<"Set samples.";
      
      LOG(INFO) << "real rows: "<<control_pts.rows() << " bests: "<<best.size();
      
      for(int i=0; i<control_pts.rows(); i++)
      {
	if (i<3)
	  continue;
	if (i>= end_id)
	  continue;
	
	for (int j=0; j<3; j++)
	{
	  //LOG(INFO)<<"i: "<<i<<" j: "<<j<<"best: "<<best[3 * (i - 3) + j];
	 
	  control_pts(i,j) = best[3 * (i - 3) + j];
	}
      }
      LOG(INFO)<<control_pts;
      
      CubicBspline bspline1(control_pts, ts, 3);
      LOG(INFO)<<"Set samples.";
      for(int i=0; i<30; i++)
      {
	visualization_->drawPath(h_path, 0.1,  Eigen::Vector4d(1, 0, 0, 1));
	visualization_->drawBspline(bspline1, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1),true, 0.12,Eigen::Vector4d(0, 1, 0, 1));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout<<"Print path..."<<std::endl;
      }
      
    }
  }
  
  void octomap_update_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (is_set_map)
      return;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);
    for(int i=0; i<out_pointcloud.points.size(); i++)
    {
      Eigen::Vector3d pd(out_pointcloud.points[i].x, out_pointcloud.points[i].y, out_pointcloud.points[i].z);
      obs_list_.push_back(pd);
    }
    std::cout << "Finish init map, Map size: " << obs_list_.size() << std::endl;
   
    astar_->setObstacle(obs_list_);
    hastar_->setObstacle(obs_list_);
    is_set_map = true;
    LOG(INFO)<<"Finish set map.";
    
    //obstacle_set_mutex.unlock();
  }
};