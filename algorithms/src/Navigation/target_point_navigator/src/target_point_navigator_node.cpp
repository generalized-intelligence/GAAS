#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "gaas_msgs/GAASNavigationPath.h"
#include "gaas_msgs/GAASGetAStarPath.h"
#include "Timer.h"

using std::endl;

class TargetPointNavigator
{
public:
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    int path_id;
    ros::ServiceClient navigation_service_client;
    ros::Subscriber currentPoseSubscriber;
    bool pose_ready = false;
    ros::Publisher targetPositionPublisher;
    geometry_msgs::PointStamped target_point;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PointStamped current_point;
    std::mutex currentPoseMutex;
    void initTargetPointNavigator(int argc,char** argv)
    {
        ros::init(argc,argv,"target_point_navigator_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        navigation_service_client = pNH->serviceClient<gaas_msgs::GAASGetAStarPath>
                ("/gaas/navigation/global_planner/astar_planning");
        currentPoseSubscriber = pNH->subscribe<geometry_msgs::PoseStamped>
                ("/gaas/localization/registration_pose",1,
                 &TargetPointNavigator::currentPoseCallback,this);
        targetPositionPublisher = pNH->advertise<geometry_msgs::PoseStamped>("/gaas/navigation/target_position",1);
        while(!pose_ready)
        {
            ros::spinOnce();
            usleep(20000);//20ms.
        }
        LOG(INFO)<<"Pose ready. Sleep 5s and wait for other nodes."<<endl;
        sleep(5);
        LOG(INFO)<<"Target point navigator stand by!"<<endl;
    }
    bool runTargetPointNavigator(double target_x,double target_y,double target_z)
    {
        LOG(INFO)<<"In runTargetPointNavigator():setting new goal:"<<target_x<<","<<target_y<<","<<target_z<<endl;
        target_point.point.x = target_x;
        target_point.point.y = target_y;
        target_point.point.z = target_z;
        bool failed = false;
        bool service_avail = navigation_service_client.waitForExistence(ros::Duration(1.0));//wait for 1s before timeout.
        if(!service_avail)
        {
            LOG(ERROR)<<"GlobalAStarPlanner service not avail! Failed."<<endl;
            return false;
        }
        while(!finishedWholePath()&&ros::ok())
        {
            //step<1> get target position in map coordinate.
            gaas_msgs::GAASGetAStarPath srv;
            auto& req = srv.request;
            {
                req.header.stamp = ros::Time::now();
                req.header.frame_id = "map";
                currentPoseMutex.lock();
                req.current_point = current_point;
                currentPoseMutex.unlock();
                req.target_point = target_point;
            }
            //    step<2> call a* planning module srv for new path to target.
            ScopeTimer srv_timer("navigation_srv_calling");
            if (navigation_service_client.call(srv))
            {
                srv_timer.watch("after srv.call()");
                if(srv.response.success)
                {
                    srv_timer.watch("navigation_srv calling finished!");
                    LOG(INFO)<<"Navigation srv client got path!"<<endl;
                    //step<3> publish new local goal.
                    if(srv.response.path.path_nodes.size()>=2)
                    {
                        auto ct_point = srv.response.path.path_nodes[1];
                        geometry_msgs::PoseStamped controller_command;
                        controller_command.header.frame_id = "map";
                        controller_command.header.stamp = ros::Time::now();
                        controller_command.pose.position.x = ct_point.x;
                        controller_command.pose.position.y = ct_point.y;
                        controller_command.pose.position.z = ct_point.z;
                        int counter = 0;
                        while(!finished_current(controller_command,srv.response.path.path_nodes.size()==2)&&ros::ok())
                        {
                            targetPositionPublisher.publish(controller_command);
                            LOG(INFO)<<"Controller command published! Point:"<<ct_point.x<<";"<<ct_point.y<<";"<<ct_point.z<<endl;
                            ros::spinOnce();
                            usleep(20000);//20 ms.
                            counter++;
                            if(counter>3000);//slower than 1min/1grid
                            {
                                LOG(WARNING)<<"Warning target_point_navigator_node detected motion too slow; replaning."<<endl;
                                break;
                            }
                        }
                        if(srv.response.path.path_nodes.size()==2)//last node
                        {
                            break;
                        }
                    }
                    else
                    {
                        LOG(INFO)<<"Path len == 1,finished!"<<endl;
                        return true;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Navigation srv a* failed!"<<endl;
                    failed=true;
                }
            }
            else
            {
                LOG(ERROR)<<"Navigation srv client calling error!"<<endl;
                failed = true;
            }
            if(failed)
            {
                LOG(INFO)<<"Global a* planner failed. Stand by."<<endl;//TODO:设置悬停,不再动.需要Mavros controller支持.
                return false;
            }
            ros::spinOnce();
            usleep(20000);//20 ms.
        }
        LOG(INFO)<<"Dist < 0.4m, finished!"<<endl;
        return true;
    }
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        LOG(INFO)<<"Enter currentPoseCallback()"<<endl;
        currentPoseMutex.lock();
        current_pose =*pose_msg;
        //TODO: current_point要不要改成geometry_msgs::Point?要header有用吗....
        current_point.point.x = pose_msg->pose.position.x;
        current_point.point.y = pose_msg->pose.position.y;
        current_point.point.z = pose_msg->pose.position.z;
        currentPoseMutex.unlock();
        pose_ready = true;
    }
private:
    bool finishedWholePath()
    {
        currentPoseMutex.lock();
        auto pt = current_point;
        currentPoseMutex.unlock();
        double dist = sqrt(pow(target_point.point.x-pt.point.x,2)+pow(target_point.point.y-pt.point.y,2)+pow(target_point.point.z-pt.point.z,2));
        if(dist<0.4)
        {
            return true;
        }
        return false;
    }
    bool finished_current(const geometry_msgs::PoseStamped& p2,bool isLast = true)
    {
        const double thres_last = 0.2;//make sure this is smaller than the thres in finishedWholePath!!!!!
        const double thres_path = 0.6;
        double THRES = isLast?thres_last:thres_path;
        currentPoseMutex.lock();
        auto pt = current_point;
        currentPoseMutex.unlock();
        double dist = sqrt(pow(p2.pose.position.x-pt.point.x,2)+pow(p2.pose.position.y-pt.point.y,2)+pow(p2.pose.position.z-pt.point.z,2));
        if(dist<THRES)
        {
            return true;
        }
        return false;
    }
};


bool CHECK_MOVEMENT(double target_x,double target_y,double target_z,TargetPointNavigator& tpn)
{
    bool result = tpn.runTargetPointNavigator(target_x,target_y,target_z);
    if(!result)
    {
        LOG(ERROR)<<"Path to target failed!"<<endl;
    }
}

typedef std::array<double,3> Target3D;

std::vector<Target3D> getTargetsByFile()
{
    std::vector<Target3D> retval;
    std::ifstream ifs("selected_targets.txt",std::ios::in);
    std::string line;
    while(std::getline(ifs,line))
    {
        std::stringstream ss(line);
        Target3D t;
        ss>>t[0];
        ss>>t[1];
        ss>>t[2];
        retval.push_back(t);
    }
    return retval;
}


int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("target_point_navigator_node");
    TargetPointNavigator tpn;
    tpn.initTargetPointNavigator(argc,argv);
    //    tpn.runTargetPointNavigator(0,0,1);
    //    tpn.runTargetPointNavigator(0,30,1);//TODO:重新建图解决这个塔高处看不见的问题.
//    bool result = false;

//    while(ros::ok())
//    {
//        result = tpn.runTargetPointNavigator(0,0,3);
//        if(!result)
//        {
//            LOG(ERROR)<<"Path to target failed!"<<endl;
//        }
//        result = tpn.runTargetPointNavigator(0,-12,3);
//        if(!result)
//        {
//            LOG(ERROR)<<"Path to target failed!"<<endl;
//        }
//        result = tpn.runTargetPointNavigator(0,-12,4);
//        if(!result)
//        {
//            LOG(ERROR)<<"Path to target failed!"<<endl;
//        }
//        result = tpn.runTargetPointNavigator(0,8,4);//用感知解决塔的上部建图时看不见的问题。
//        if(!result)
//        {
//            LOG(ERROR)<<"Path to target failed!"<<endl;
//        }
//    }
    auto targets = getTargetsByFile();
    if(targets.size() == 0)
    {
        LOG(ERROR)<<"ERROR: Empty targets file. Target point navigator will quit."<<endl;
        exit(-1);
    }
    while(ros::ok())
    {
        for(auto t:targets)
        {
            CHECK_MOVEMENT(t[0],t[1],t[2],tpn);
        }
    }
    return 0;
}
