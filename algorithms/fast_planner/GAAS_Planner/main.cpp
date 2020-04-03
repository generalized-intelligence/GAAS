#include <iostream>
#include "astar.h"
//#include "hybrid_astar.h"
#include "test_octomap.hpp"
#include <map>
#include <ros/ros.h>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <time.h>
#include <glog/logging.h>
#include "plan_enviroment/enviroment.h"
#include "utils/path_visulization.h"
#include <fstream>
#include <path_optimization/bspline_nlp.h>

int main(int argc, char **argv) 
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  
  ros::init(argc, argv, "gi_navigator_node");
  
  
  std::ifstream infile;
  infile.open("../test_point.txt");
  if (!infile.is_open())
	LOG(ERROR) << "open file failure";
  
  double x, y, z;
  infile >> x >> y >> z;
  infile.close();
  
  
  ros::M_string remap_str;
  remap_str.insert(make_pair<std::string, std::string>("xx/esdf_map_in",
						       "/my_robot/esdf_map"));
  ros::NodeHandle nh("~", remap_str);
  ros::NodeHandle nh_private("~", remap_str);
  
  PathVisulization pvis(nh);
  
  SdfEnviroment sdf(nh, nh_private);
  Eigen::Vector3d pos(x,y,z);
  std::vector<Eigen::Vector3d> pos_list;
  pos_list.push_back(pos);
  Eigen::Vector3d v(0,0,0);
  
  while(ros::ok())
  {
    double d = sdf.getDistance(pos);
    LOG(INFO)<<"Oh fuck: "<< sdf.getDistance(pos);
    if (d!=1000)
      break;
//     LOG(INFO)<<"Oh fuck: "<< sdf.getDistance(pos);
    //LOG(INFO)<<"Oh G fuck: "<< sdf.getDistanceAndGradiend(pos, v);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    pvis.displaySphereList(pos_list, 0.2 ,Eigen::Vector4d(1, 0, 0, 1));
    ros::spinOnce();
  }
  
  
  ros::Rate rate(50.0);
  DLOG(INFO)<<"Start test.";
  TestOctomap t(&nh);
  std::cout<<"Finish Init test."<<std::endl;
//   while (!t.is_set_map)
//   {
//     std::cout<<"Sleep"<<std::endl;
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   }
  DLOG(INFO)<<"Finish set map.";
  t.searchHybrid(sdf);
  DLOG(INFO)<<"Finish test.";
  ros::spin();
  return 0;
  

}

