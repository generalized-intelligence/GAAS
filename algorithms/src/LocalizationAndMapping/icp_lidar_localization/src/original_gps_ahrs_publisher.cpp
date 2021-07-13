#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>

#include "GPS_AHRS_sync.h"
#include "ndt_algo.h"
#include "Timer.h"

#include <Eigen/Geometry>

class GPS_AHRS_Publisher;
void gps_ahrs_sync_callback_wrapper(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom);

GPS_AHRS_Publisher* pPublisher;
class GPS_AHRS_Publisher
{
public:

    GPS_AHRS_Synchronizer gas;
    NDTAlgo* p_algo;
    ros::Publisher gps_ahrs_pose_pub;
    std::shared_ptr<ros::NodeHandle> pNH;


    void init_node(int argc,char** argv)
    {
        ros::init(argc,argv,"gps_ahrs_publisher_node");
        this->pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        this->gps_ahrs_pose_pub = this->pNH->advertise<geometry_msgs::PoseStamped>("/gaas/localization/gps_ahrs_original",1);

        //load gps info.
        this->p_algo = new NDTAlgo(&this->gas);
        p_algo->loadPCDMap();
        LOG(ERROR)<<"[original gps ahrs publisher] PCD Map loaded!"<<endl;

        message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(*pNH, "/mavros/global_position/raw/fix", 1);
        //message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/mavros/global_position/global", 1);
        message_filters::Subscriber<nav_msgs::Odometry> ahrs_sub(*pNH, "/mavros/local_position/odom", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry> MySyncPolicy;
        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), gps_sub, ahrs_sub);
        sync.registerCallback(boost::bind(&gps_ahrs_sync_callback_wrapper, _1, _2));
        while(ros::ok())
        {
            ros::spinOnce();
        }
    }
    void gps_ahrs_sync_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom)// 同步gps和ahrs.
    {
        LOG(ERROR)<<"Enter GPS_AHRS_CALLBACK!!!"<<endl;
        const sensor_msgs::NavSatFix& gps = *gps_msg;
        const nav_msgs::Odometry& ahrs = *odom;
        Eigen::Matrix4f gps_ahrs_initial_guess;
        {
            double x,y,z;
            this->p_algo->map_gps_info.getRelativeXYZFromLonLatAltInNWUCoordinate(gps.longitude,gps.latitude,gps.altitude,x,y,z);
            LOG(INFO)<<"GPS relative XYZ:"<<x<<";"<<y<<";"<<z<<endl;
            auto orient = ahrs.pose.pose.orientation;
            Eigen::Quaternionf original;
            original.x() = orient.x;
            original.y() = orient.y;
            original.z() = orient.z;
            original.w() = orient.w;

            //P_nwu = T_nwu_enu*T_enu_body*P_body
            Eigen::Matrix3f R_nwu_enu;
            R_nwu_enu<<0,1,0, -1,0,0 ,0,0,1; //plane right.
            Eigen::Matrix3f NWU_R = (R_nwu_enu*original.toRotationMatrix());

            Eigen::Quaternionf NWU_orient(NWU_R);
            LOG(INFO)<<"NWU coord quaternion initial guess = "<<NWU_orient.x()<<","<<NWU_orient.y()<<","<<NWU_orient.z()<<","<<NWU_orient.w()<<endl;

            gps_ahrs_initial_guess.block(0,0,3,3) = NWU_R;
            Eigen::Vector3f xyz_(x,y,z);

            gps_ahrs_initial_guess(0,3) = xyz_(0);
            gps_ahrs_initial_guess(1,3) = xyz_(1);
            gps_ahrs_initial_guess(2,3) = xyz_(2) - this->p_algo->lidar_height_compensation;
            gps_ahrs_initial_guess(3,3) = 1;

        }

        {
            Eigen::Matrix3f rotmat = gps_ahrs_initial_guess.block(0,0,3,3);
            Eigen::Quaternionf quat(rotmat);
            geometry_msgs::PoseStamped ndt_pose_msg;

            ndt_pose_msg.header.frame_id="map";//地图坐标系的定位.
            ndt_pose_msg.header.stamp = gps.header.stamp;//与gps同步.

            auto& pos = ndt_pose_msg.pose.position;
            auto& orient = ndt_pose_msg.pose.orientation;
            orient.x = quat.x();
            orient.y = quat.y();
            orient.z = quat.z();
            orient.w = quat.w();

            pos.x = gps_ahrs_initial_guess(0,3);
            pos.y = gps_ahrs_initial_guess(1,3);
            pos.z = gps_ahrs_initial_guess(2,3);

            this->gps_ahrs_pose_pub.publish(ndt_pose_msg);
            LOG(INFO)<<"GPS AHRS pose published!"<<endl;
        }
    }
};
void gps_ahrs_sync_callback_wrapper(const sensor_msgs::NavSatFixConstPtr& gps_msg, const nav_msgs::OdometryConstPtr& odom)
{
    pPublisher->gps_ahrs_sync_callback(gps_msg,odom);
}


int main(int argc,char** argv)
{
//    FLAGS_alsologtostderr = 1;
//    google::InitGoogleLogging("gps_ahrs_publisher_node");
//    GPS_AHRS_Publisher gap;
//    gap.init_node(argc,argv);
    return 0;
}

