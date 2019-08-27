#include <iostream>
#include <bits/stdc++.h> 
#include <set> 
#include <ros/ros.h>
#include <vector>
#include <map>
#include <string.h>
#include <list>
#include <algorithm>
#include <math.h>
#include "astar/driver.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <thread>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapWithPose.h>
#include "DiscreteGridUtils.h"
#include <mutex>
#include "path_optimization/path_pruning.h"
#include "astar/astar.h"
#include <chrono>
#include <pthread.h>

using namespace std; 
typedef pair<double,  pair<double, double> > Pair; 
typedef pair<string, Pair> sPair;
typedef set <Pair> setpair;
//typedef pair<bool, double> bdpair;

/*
template <typename T,typename U, typename R>                                                   
std::pair<T,pair<U,R> > operator+(const std::pair<T,pair<U,R> > & l,const std::pair<T,pair<U,R> > & r)
{   
    return {l.first+r.first,{l.second.first+r.second.first, l.second.second+r.second.second}};                                                                       
}*/


double square(Pair pos1, Pair pos2)
{
    return (abs(pos1.first - pos2.first)*abs(pos1.first - pos2.first) +  abs(pos1.second.first - pos2.second.first)* abs(pos1.second.first - pos2.second.first) + abs(pos1.second.second - pos2.second.second)*abs(pos1.second.second - pos2.second.second));
}


class Controller
{
    public:
    geometry_msgs::PoseStamped set_pose(double x=0, double y=0, double z=0, bool abs_mode = true)
    {
       geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        cout<<"(x,y,z):"<<"("<<x<<","<<y<<","<<z<<")"<<endl;

        if(!abs_mode)
        {
            pose.header.frame_id = "base_link";
        }

        return pose;

    }

    void takeoff_ned_xyz_relative(double x, double y, double z, ros::Publisher local_setposition_pub)
    {
        local_setposition_pub.publish(set_pose(x,y,z,false));
    }

    void takeoff_ned_xyz_abs(double x, double y, double z, ros::Publisher local_setposition_pub)
    {
        local_setposition_pub.publish(set_pose(x,y,z));
    }

    void turn_relative_to_takeoff_abs(std_msgs::Float32 degree, ros::Publisher local_setorientation_pub)
    {
        local_setorientation_pub.publish(degree);
    }

    void mav_move(double x, double y, double z, ros::Publisher local_setposition_pub, ros::Publisher local_setorientation_pub, bool abs_mode = true)
    {
        //NED to NEU
        if(abs_mode)
            takeoff_ned_xyz_abs(-y,x,z, local_setposition_pub);
        else
            takeoff_ned_xyz_relative(-y,x,z,local_setposition_pub);
    }
};

class Navigator
{
    public:
    string mavros_state;
    int curr_command_id;
    int prev_command_id;
    Pair cur_target_position;
    Pair local_pose;
    Pair point;

    int task_id;
    
    geometry_msgs::PoseStamped slam_pose;
    geometry_msgs::PoseStamped vision_target;
    sensor_msgs::PointCloud current_point_cloud;
    sensor_msgs::PointCloud out_pointcloud; 
    mavros_msgs::State mavros_state_msg;


    set<Pair> obs_set;
    Driver d;
    Controller c;
    DiscreteGridUtils dg;
    mutex obstacle_set_mutex;
    mutex nav_command_mutex;
    vector<Pair> path;
    PathPruning path_prune;
    double obstacle_distance;
    bool found_path;
    set<Pair> obstacle_map;

    ros::Publisher path_plan_pub;
    ros::Publisher navigator_status_pub;
    ros::Publisher local_setposition_pub;
    ros::Publisher local_setorientation_pub;

    ros::Subscriber slam_sub;
    ros::Subscriber vision_target_sub;
    ros::Subscriber point_cloud_sub;
    ros::Subscriber octomap_cells_vis;
    ros::Subscriber local_pose_sub;
    ros::Subscriber mavros_sub;
    ros::NodeHandle nh;

    Navigator(ros::NodeHandle* n, Pair p):nh(*n)
    {
        
        //DiscreteGridUtils dg(0.2);
        DiscreteGridUtils dg(0.2);
        ros::Rate rate(50.0);
        Driver d;
        Controller c;
        ros::Time::init();
        
        mavros_state = "OFFBOARD";
       
        //Pair local_pose;
        obstacle_set_mutex.lock();
        nav_command_mutex.lock();
        thread t1(&Navigator::ros_thread, this);
        t1.detach();
        
        obstacle_distance = 8;
        

        
        
         navigator_status_pub = nh.advertise<std_msgs::String>
            ("/gi/navigator_status", 10);
         path_plan_pub = nh.advertise<visualization_msgs::MarkerArray>
            ("/gi/navi_path_plan", 10);
         local_setposition_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("gi/set_pose/position", 10);
         local_setorientation_pub = nh.advertise<std_msgs::Float32>
            ("gi/set_pose/orientation", 10);
            set_target_postion(p);
            //cout<<"current point is "<<get_current_pose().first<<" "<<get_current_pose().second.first<<" "<<get_current_pose().second.second<<endl;
            PathPruning path_prune(obstacle_distance);

    }

    void keep_navigating()
    {
        cout<<"In keep navigating"<<"   "<<mavros_state<<endl;
        while(mavros_state == "OFFBOARD" && ros::ok())
        {
            Pair relative_pos = make_pair(0,make_pair(0,0));
            Pair end_pos = get_latest_target();

            Pair current_pos = get_current_pose();

            while(current_pos != end_pos && !navi_task_terminated() && ros::ok())
            {
                current_pos = get_current_pose();
                
                cout<<"Move 1 step"<<endl;

                obstacle_map = d.get_obstacles_around();
                cout<<"From "<<"("<<current_pos.first<<","<<current_pos.second.first<<","<<current_pos.second.second<<")"<<endl;
                auto start = chrono::high_resolution_clock::now(); 
                A_star algo(end_pos);
                path = algo.find_path(current_pos, d.get_obstacles_around());
                //auto stop = chrono::high_resolution_clock::now(); 
                //auto duration = chrono::duration_cast<chrono::microseconds>(stop - start); 
                //cout<<"A* time cost: "<< duration.count() <<endl;
                //cout<<"yo.."<<endl;
                if(path.empty())
                {
                    cout<<"No Path exists"<<endl;
                }
                else
                {
                    //Path found
                    vector<Pair> collinear_check_path = path_prune.remove_collinear_points(path);
                    vector<Pair> bresenham_check_path = path_prune.path_pruning_bresenham3d(collinear_check_path, obstacle_map);


                    //publish raw path plan
                    visualization_msgs::MarkerArray m_arr;
                    int marr_index = 0;

                    for(int i=0; i<path.size(); i++)
                    {
                        point = dg.discrete_to_continuous_target(path[i]);
                        visualization_msgs::Marker mk;
                        mk.header.frame_id = "map";
                        mk.action = mk.ADD;
                        mk.id = marr_index;
                        marr_index++;
                        mk.color.r = 1.0;
                        mk.color.a = 1.0;
                        mk.type = mk.CUBE;
                        mk.scale.x = 0.3;
                        mk.scale.y = 0.3;
                        mk.scale.z = 0.3;
                        mk.pose.position.x = path[i].first;
                        mk.pose.position.y = path[i].second.first;
                        mk.pose.position.z = path[i].second.second;
                        m_arr.markers.push_back(mk);
                    }
                    path_plan_pub.publish(m_arr);

                    for(int j=0; j<bresenham_check_path.size(); j++)
                    {
                        path_plan_pub.publish(m_arr);
                        if(navi_task_terminated())
                        {
                            break;
                        }
                        cout<<"current_pos: "<<"("<<current_pos.first<<","<<current_pos.second.first<<","<<current_pos.second.second<<")"<<endl;
                        Pair next_pos = bresenham_check_path[j];
                        relative_pos = make_pair(next_pos.first - current_pos.first, make_pair(next_pos.second.first - current_pos.second.first,next_pos.second.second - current_pos.second.second));
                        cout<<"next_move: "<<"("<<next_pos.first<<","<<next_pos.second.first<<","<<next_pos.second.second<<")"<<endl;
                        cout<<"relative_move: "<<"("<<relative_pos.first<<","<<relative_pos.second.first<<","<<relative_pos.second.second<<")"<<endl;

                        if(!algo.is_valid(next_pos, d.get_obstacles_around()))
                        {
                            cout<<"Path not valid!";
                            break;
                        }
                        current_pos = next_pos;

                        //axis transform
                        Pair relative_pos_new = make_pair(-relative_pos.first, make_pair(-relative_pos.second.first, relative_pos.second.second));

                        cout<<"mav_move() input: relative pos = "<<"("<<next_pos.first<<","<<next_pos.second.first<<","<<next_pos.second.second<<")"<<endl;
                        Pair temp = dg.discrete_to_continuous_target(next_pos);
                        c.mav_move( temp.first,  temp.second.first,  temp.second.second, local_setposition_pub, local_setorientation_pub, true);
                        current_pos = get_current_pose();
                        sleep(2);
                        Pair predict_move = current_pos + relative_pos;
                        cout<<"Predict_move: "<<"("<<predict_move.first<<","<<predict_move.second.first<<","<<predict_move.second.second<<")"<<endl;

                        if(!algo.path_is_valid(bresenham_check_path, d.get_obstacles_around()))
                        {
                            cout<<"Path conflict detected!"<<endl;
                            break;
                        }
                    }
                }


            }
            //sleep(0.05)

        }
        cout<<"Mavros not in OFFBOARD mode, Disconnected!"<<endl;
    }



    void ros_thread()
    {
        cout<<"ros_thread spawn!!!"<<endl;
         slam_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/gi/slam_output/pose", 10, &Navigator::slam_pose_callback, this);
           // cout<<"ros_thread !!!"<<endl;
         vision_target_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/gi/visual_target/pose", 10, &Navigator::vision_target_callback, this);
         point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud>
            ("/camera/left/point_cloud", 10, &Navigator::point_cloud_callback, this);
         octomap_cells_vis = nh.subscribe<sensor_msgs::PointCloud2>
            ("/octomap_point_cloud_centers", 10, &Navigator::octomap_update_callback, this);
         local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, &Navigator::local_pose_callback, this);
        mavros_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, &Navigator::mavros_state_callback, this);


        //self.set_status(status.INITIALIZED)

        ros::spinOnce();

    }                                                                  

    
    void slam_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        cout<<"slam_pose_callback  "<<endl;
        slam_pose = *msg;
    }


    void vision_target_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {cout<<"vision_target_callback  "<<endl;
        vision_target = *msg;
    }


    void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg)
    {cout<<"mavros_state_callback  "<<endl;
        mavros_state_msg = *msg;
        mavros_state = mavros_state_msg.mode;
        
    }


    void point_cloud_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
    {cout<<"point_cloud_callback  "<<endl;
        current_point_cloud = *msg;
    }


    void octomap_update_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {cout<<"octomap_update_callback  "<<endl;
        
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);
        for(int i=0; i<out_pointcloud.points.size(); i++)
        {
            Pair p = make_pair(out_pointcloud.points[i].x, make_pair(out_pointcloud.points[i].y, out_pointcloud.points[i].z));
            obs_set.insert(dg.continuous_to_discrete(p));
        }
        obstacle_set_mutex.unlock();
        d.set_obstacle_set(obs_set);
        //obstacle_set_mutex.release()
    }


    void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {//cout<<"local_pose_callback  "<<endl;
        geometry_msgs::PoseStamped temp = *msg;
        //cout<<"current pos is "<<temp.pose.position.x<<" "<<temp.pose.position.y<<" "<<temp.pose.position.z<<endl;
        Navigator::local_pose = dg.continuous_to_discrete(make_pair(temp.pose.position.x, make_pair(temp.pose.position.y, temp.pose.position.z))); 
    }

    Pair get_local_pose()
    {
        return local_pose;
    }

    Pair get_current_pose()
    {
        return get_local_pose();
    }


    void set_target_postion(Pair target_position)
    {
        found_path = true;
        cur_target_position = target_position;
        cout<<"target position set"<<endl;
    }
    Pair get_latest_target()
    {
        return cur_target_position;
    }

    bool navi_task_terminated()
    {
        if(dist(local_pose, cur_target_position) < 0.25)
        {
            return true;
        }
        else
        {
            return false;
        } 
    }

    double dist(Pair pt1, Pair pt2)
    {
        return(sqrt(pow((pt1.first - pt2.first),2)+pow((pt1.second.first - pt2.second.first),2)+pow((pt1.second.second - pt2.second.second),2)));
    }


};


//mavros_msgs::State current_state;
//void state_cb(const mavros_msgs::State::ConstPtr& msg){cout<<"state call back called"<<endl;
//    current_state = *msg;
//}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "gi_navigator_node");
    ros::NodeHandle n;
    ros::Rate rate(50.0);
    //ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
    //        ("mavros/state", 10, state_cb);
    Navigator nav(&n,make_pair(80,make_pair(0,2)));
    //cout<<"going to set target position"<<endl;
    //nav.set_target_postion(make_pair(80,make_pair(0,2)));
    nav.keep_navigating();
    ros::spin();

    return 0;
}
