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


int main(int argc, char **argv) 
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  
  
  ros::init(argc, argv, "gi_navigator_node");
  ros::NodeHandle n;
  ros::Rate rate(50.0);
  DLOG(INFO)<<"Start test.";
  TestOctomap t(&n);
  std::cout<<"Finish Init test."<<std::endl;
  while (!t.is_set_map)
  {
    std::cout<<"Sleep"<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  DLOG(INFO)<<"Finish set map.";
  t.searchHybrid();
  DLOG(INFO)<<"Finish test.";
  ros::spin();
  return 0;
  

}

