//Here we implement a whole ros app to publish attitude calc by a simple AHRS.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
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
uNavAHRS* pAHRS;
ros::Publisher* pvis_attitude_pub;
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
    m->scale.y = 0.5;
    m->scale.z = 0.5;
    m->action = visualization_msgs::Marker::ADD;
    m->type = visualization_msgs::Marker::ARROW;
    return m;
} 


void onReceivedIMUandMagnetMessage(const sensor_msgs::Imu& imu_msg,const sensor_msgs::MagneticField& mag_msg)
{
    pAHRS->update(imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z,
		imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z,
		mag_msg.magnetic_field.x,mag_msg.magnetic_field.y,mag_msg.magnetic_field.z);
    float x,y,z,w;
    pAHRS->getQuaternion(&x,&y,&z,&w);
    shared_ptr<visualization_msgs::Marker> mark(make_marker_from_quaternion(x,y,z,w));
    //publish.
    pvis_attitude_pub->publish(*mark);
}
int main(int argc,char** argv)
{
    pAHRS = new(uNavAHRS);
    ros::init(argc,argv,"AHRS");
    ros::NodeHandle nh;
    string mag_topic_name("/mavros/imu/mag");
    string imu_topic_name("/mavros/imu/data");//considered imu bias.
    message_filters::Subscriber<sensor_msgs::Imu> imusub(nh,imu_topic_name,10);
    message_filters::Subscriber<sensor_msgs::MagneticField> mag_sub(nh,mag_topic_name,10);
    auto pub = nh.advertise<visualization_msgs::Marker>("/AHRS_vis_marker",10);
    pvis_attitude_pub = &pub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,sensor_msgs::MagneticField> sync_pol_t;
    message_filters::Synchronizer<sync_pol_t> sync(sync_pol_t(10),imusub,mag_sub);
    sync.registerCallback(
				//boost::bind(
			onReceivedIMUandMagnetMessage);//,_1,_2));
    ros::spin();
    return 0;
}




