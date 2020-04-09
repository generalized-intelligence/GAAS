//TODO:display path in rviz


#include "utils/path_visulization.h"

PathVisulization::PathVisulization(ros::NodeHandle& nh): nh_(nh)
{
  traj_pub = nh_.advertise<visualization_msgs::Marker>("/gaas/trajectory", 10);
}


void PathVisulization::displaySphereList(std::vector< Eigen::Vector3d > list, double resolution, Eigen::Vector4d color, int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
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
  displaySphereList(path, resolution, color, 2);
}

void PathVisulization::drawBspline(CubicBspline bspline, double size, Eigen::Vector4d color, bool show_ctrl_pts, double size2, Eigen::Vector4d color2)
{
  LOG(INFO)<<"Start draw traj";
  std::vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getValidTimeSpan(tm, tmp);
  LOG(INFO)<<"Time span"<<tm;
  for (double t = tm; t <= tmp; t += 0.01)
  {
    //LOG(INFO)<<"Get time: "<<t;
    Eigen::Vector3d pt = bspline.deBoorCox(t);
    traj_pts.push_back(pt);
    //LOG(INFO)<<"Get pt: "<<pt;
  }
  displaySphereList(traj_pts, size, color, 3);
  
  LOG(INFO)<<"Finish draw traj";

  // draw the control point
  if (!show_ctrl_pts)
    return;

  Eigen::MatrixXd ctrl_pts = bspline.getControlPoints();

  std::vector<Eigen::Vector3d> ctp;
  for (int i = 0; i < int(ctrl_pts.rows()); ++i)
  {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }
  displaySphereList(ctp, size2, color2, 4);
}
