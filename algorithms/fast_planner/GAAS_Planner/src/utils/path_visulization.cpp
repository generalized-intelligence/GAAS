//TODO:display path in rviz


#include "utils/path_visulization.h"

PathVisulization::PathVisulization(ros::NodeHandle& nh): nh_(nh)
{
  traj_pub = nh_.advertise<visualization_msgs::Marker>("/gaas/trajectory", 10);
}


void PathVisulization::displaySphereList(std::vector< Eigen::Vector3d > list, double resolution, Eigen::Vector4d color)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = 1;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0, mk.pose.orientation.w = 1.0;
  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0), pt.y = list[i](1), pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
}

void PathVisulization::drawPath(std::vector< Eigen::Vector3d > path, double resolution, Eigen::Vector4d color)
{
  displaySphereList(path, resolution, color);
}
