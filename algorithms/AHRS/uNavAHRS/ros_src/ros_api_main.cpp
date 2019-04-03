//Here we implement a whole ros app to publish attitude calc by a simple AHRS.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include "uNavAHRS.h"
#include <memory>
using namespace std;

int marker_id = 0;
double last_info_time = -1.0;
uNavAHRS* pAHRS;
ros::Publisher* pvis_attitude_pub;
const bool print_fc_ahrs = true;
shared_ptr<visualization_msgs::Marker> make_marker_from_quaternion(double x,double y,double z,double w)
{
    shared_ptr<visualization_msgs::Marker> m(new visualization_msgs::Marker());
    m->header.frame_id = "/map";
    m->id = marker_id;
    m->color.a = 0.5;
    m->color.r = 1.0;
    m->pose.orientation.x = x;
    m->pose.orientation.y = y;
    m->pose.orientation.z = z;
    m->pose.orientation.w = w;
    m->scale.x = 0.5;
    m->scale.y = 0.1;
    m->scale.z = 0.5;
    m->action = visualization_msgs::Marker::ADD;
    m->type = visualization_msgs::Marker::ARROW;
    return m;
} 
float degtorad(double deg)
{
    return 3.14159*(deg/180.0);
}

void onReceivedIMUandMagnetMessage(const sensor_msgs::Imu& imu_msg,const sensor_msgs::MagneticField& mag_msg)
{
//Here pay attention:
//We do assert that 
//	1:the timestamp is more accurate than unix time,and the accuracy of imu is far more important than magnetic values;so here we use the timestamp of imu msg.
//	2:And while initializing,we do not care about the accuracy of time,just use micros().

    /*
    double current_time = imu_msg.header.stamp.toSec();
    float dt = -1;
    if(last_info_time>0)
    {
        dt = (float) current_time-last_info_time;
        last_info_time = current_time;
    }
    else
    {
        cout<<"Time error!"<<endl;
        last_info_time = current_time;
        return;
    }*/
    auto ang = imu_msg.angular_velocity;
    bool filter_ready = pAHRS->update(ang.x,ang.y,ang.z,
		imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z,
		mag_msg.magnetic_field.x,mag_msg.magnetic_field.y,mag_msg.magnetic_field.z);
			//,dt);
    if(filter_ready)
    {
      float x,y,z,w;
      pAHRS->getQuaternion(&x,&y,&z,&w);
      cout<<"x,y,z,w:"<<x<<","<<y<<","<<z<<","<<w<<"."<<endl;
      shared_ptr<visualization_msgs::Marker> mark(make_marker_from_quaternion(x,y,z,w));
      //publish.
      pvis_attitude_pub->publish(*mark);
    }
    else
    {
        cout<<"Filter initializing.Do not move IMU."<<endl;
    }
}
void onReceivedFC_AHRS_Message(const nav_msgs::Odometry& ahrs_msg)
{
    //cout<<"received fc ahrs."<<endl;
    shared_ptr<visualization_msgs::Marker> mark(make_marker_from_quaternion(
	ahrs_msg.pose.pose.orientation.x,ahrs_msg.pose.pose.orientation.y,ahrs_msg.pose.pose.orientation.z,ahrs_msg.pose.pose.orientation.w
));
    mark->color.b =1.0;
    mark->color.r = 0;
    mark->id = 1;
    pvis_attitude_pub->publish(*mark);
}
int main(int argc,char** argv)
{
    pAHRS = new(uNavAHRS);
    pAHRS->setInitializationDuration(10000000); //take 10 s init filter params.
    ros::init(argc,argv,"AHRS");
    ros::NodeHandle nh;
    string mag_topic_name("/mavros/imu/mag");
    string imu_topic_name("/mavros/imu/data");//considered imu bias.
    message_filters::Subscriber<sensor_msgs::Imu> imusub(nh,imu_topic_name,10);
    message_filters::Subscriber<sensor_msgs::MagneticField> mag_sub(nh,mag_topic_name,10);
    auto pub = nh.advertise<visualization_msgs::Marker>("/AHRS_vis_marker",10);
    pvis_attitude_pub = &pub;
    ros::Subscriber sub_fc_ahrs = nh.subscribe("/mavros/local_position/odom",10,onReceivedFC_AHRS_Message);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,sensor_msgs::MagneticField> sync_pol_t;
    message_filters::Synchronizer<sync_pol_t> sync(sync_pol_t(10),imusub,mag_sub);
    sync.registerCallback(
				//boost::bind(
			onReceivedIMUandMagnetMessage);//,_1,_2));
    ros::spin();
    return 0;
}




