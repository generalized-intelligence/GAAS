#include "../../basic_state_libs/src/flight_controller_state.h"
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt32.h>

#include <glog/logging.h>

using std::endl;

class PX4StateReporter
{
public:
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    ros::Subscriber px4_state_sub, px4_battery_state_sub, px4_GPSNavSatFix_sub,
                      px4_imutemperature_sub, px4_imu_filtered_sub, px4_velocity_estimation_sub,
                      px4_gps_satellites_count_sub;
    bool px4_state_init = false, px4_battery_state_init = false, px4_GPSNavSatFix_init = false,
                      px4_imutemperature_init = false, px4_imu_filtered_init = false, px4_velocity_estimation_init = false,
                      px4_gps_satellites_count_init = false;
    bool whole_node_init = false;

    FlightControllerState curr_state;//基本信息靠这四个subscriber更新;可选部分更新到optional_info里面去.
    FlightControllerState getStateFromPX4State(const mavros_msgs::State& px4_state);
    std::shared_ptr<ros::Publisher> pFCStatePub;

    void publishPX4ControllerState(const ros::Time& stamp)
    {
        if(!whole_node_init)//TODO.适配真机无gps调试...
        {
            LOG(WARNING)<<"Node not initialized; can not publish state."<<endl;
            return;
        }
        gaas_msgs::GAASSystemManagementFlightControllerState gaas_state;
        gaas_state.header.stamp = stamp;
        gaas_state.header.frame_id = "gaas_system_management";
        curr_state.toROSMsg(gaas_state);
        pFCStatePub->publish(gaas_state);
        LOG(INFO)<<"px4 flight controller state published!"<<endl;
    }
    void px4StateCallback(const mavros_msgs::StateConstPtr& p_px4_state_msg)
    {
        px4_state_init = true;
        const mavros_msgs::State& state_msg = *p_px4_state_msg;
        curr_state.fc_connected = state_msg.connected;
        curr_state.armed = state_msg.armed;
        if(state_msg.mode == "OFFBOARD")
        {
            curr_state.flight_mode = curr_state.OFFBOARD_MODE;
        }
        else if (state_msg.mode == "POSCTL"||state_msg.mode == "HOLD")
        {
            curr_state.flight_mode = curr_state.POSCTL_MODE;
        }
        else if (state_msg.mode == "STABILIZED")
        {
            curr_state.flight_mode = curr_state.STABILIZE_MODE;
        }
        else if (state_msg.mode == "ACRO"||state_msg.mode == "MANUAL")
        {
            curr_state.flight_mode = curr_state.MANUAL_MODE;
        }
        else if (state_msg.mode == "AUTO.MISSION" || state_msg.mode == "AUTO.LOITER"||state_msg.mode == "AUTO.RTL" || state_msg.mode == "AUTO.LAND"||state_msg.mode =="AUTO.RTGS"||state_msg.mode == "AUTO.READY"||state_msg.mode =="AUTO.TAKEOFF")
        {
            curr_state.flight_mode = curr_state.FLIGHCONTROLLER_AUTO_MODE; // 飞控自动模式,飞行器不受GAAS控制.
        }
        publishPX4ControllerState(p_px4_state_msg->header.stamp);
    }
    void px4BatteryStateCallback(const sensor_msgs::BatteryStateConstPtr& p_battery_msg)
    {
        px4_battery_state_init = true;
        curr_state.battery_soc_percentage = p_battery_msg->percentage;
        std::stringstream ss;
        ss<<p_battery_msg->voltage;
        curr_state.optional_info["battery_voltage"] = ss.str();

        std::stringstream ss2;
        ss2<<p_battery_msg->current;
        curr_state.optional_info["battery_current"] = ss2.str();
    }
    void px4GPSNavSatFixCallback(const sensor_msgs::NavSatFixConstPtr& p_NavSatFix_msg)
    {
        px4_GPSNavSatFix_init = true;
        curr_state.gps_lon_dop = p_NavSatFix_msg->position_covariance.at(0);
        curr_state.gps_lat_dop = p_NavSatFix_msg->position_covariance.at(4);
        curr_state.gps_alt_dop = p_NavSatFix_msg->position_covariance.at(8);

        curr_state.gps_longitude = p_NavSatFix_msg->longitude;
        curr_state.gps_latitude = p_NavSatFix_msg->latitude;
        curr_state.gps_altitude = p_NavSatFix_msg->altitude;
        curr_state.gps_fix_status = p_NavSatFix_msg->status.status;
    }
    void px4IMUTemperatureCallback(const sensor_msgs::TemperatureConstPtr& p_temp)//px4 温度报告
    {
        px4_imutemperature_init = true;
        curr_state.temperature_celsius = p_temp->temperature;
    }
    void px4IMUFilteredCallback(const sensor_msgs::ImuConstPtr& p_imu_msg)
    {
        px4_imu_filtered_init = true;
        curr_state.imu_ax = p_imu_msg->linear_acceleration.x;
        curr_state.imu_ay = p_imu_msg->linear_acceleration.y;
        curr_state.imu_az = p_imu_msg->linear_acceleration.z;

        curr_state.imu_wx = p_imu_msg->angular_velocity.x;
        curr_state.imu_wy = p_imu_msg->angular_velocity.y;
        curr_state.imu_wz = p_imu_msg->angular_velocity.z;

        curr_state.imu_filtered_qx = p_imu_msg->orientation.x;
        curr_state.imu_filtered_qy = p_imu_msg->orientation.y;
        curr_state.imu_filtered_qz = p_imu_msg->orientation.z;
        curr_state.imu_filtered_qw = p_imu_msg->orientation.w;
        publishPX4ControllerState(p_imu_msg->header.stamp);//加快发布的频率让HUD更流畅.
    }
    void px4VelocityEstimationCallback(const geometry_msgs::TwistStampedConstPtr& p_velocity_local)// /mavros/local_position/velocity_body  / velocity_local; //TODO:这个是什么坐标系？
    {//TODO:确认坐标系约定.
        px4_velocity_estimation_init=true;
        curr_state.vehicle_vx = p_velocity_local->twist.linear.x;
        curr_state.vehicle_vy = p_velocity_local->twist.linear.y;
        curr_state.vehicle_vz = p_velocity_local->twist.linear.z;
    }
    void px4GPSSatellitesCountCallback(const std_msgs::UInt32ConstPtr& p_satellites_count_msg) // /mavros/global_position/raw/satellites | std_msgs/UInt32
    {
        px4_gps_satellites_count_init=true;
        curr_state.gps_avail_satellite_count = p_satellites_count_msg->data;
    }
    void waitForAll()//初始化所有状态
    {
        LOG(INFO)<<"In waitForAll()..."<<endl;
        while(
              !(px4_state_init && px4_battery_state_init &&
                px4_GPSNavSatFix_init &&
                px4_imutemperature_init && px4_imu_filtered_init && px4_velocity_estimation_init &&
                px4_gps_satellites_count_init)&&ros::ok()
              )
        {
            ros::spinOnce();
            usleep(20000);//20ms.
        }
        whole_node_init = true;
        if(ros::ok())
        {
            LOG(INFO)<<"px4_state_reporter waitForAll() finished successfully."<<endl;
        }
        else
        {
            LOG(INFO)<<"Initialization failed and quit."<<endl;
            exit(-1);
        }
    }
    void initROSAndAllSubscribers(int argc,char** argv)
    {
        //px4_state_sub,px4_battery_state_sub,px4_GPSNavSatFix_sub,
        //px4_imutemperature_sub,px4_imu_filtered_sub,px4_velocity_estimation_sub,
        //px4_gps_satellites_count_sub;
        ros::init(argc,argv,"px4_state_reporter_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        px4_state_sub = pNH->subscribe<mavros_msgs::State>("/mavros/state",1,&PX4StateReporter::px4StateCallback,this);
        px4_battery_state_sub = pNH->subscribe<sensor_msgs::BatteryState>("/mavros/battery",1,&PX4StateReporter::px4BatteryStateCallback,this);
        px4_GPSNavSatFix_sub = pNH->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix",1,&PX4StateReporter::px4GPSNavSatFixCallback,this);//global_position/global不可用.
        px4_imutemperature_sub = pNH->subscribe<sensor_msgs::Temperature>("/mavros/imu/temperature_imu",1,&PX4StateReporter::px4IMUTemperatureCallback,this);
        px4_imu_filtered_sub = pNH->subscribe<sensor_msgs::Imu>("/mavros/imu/data",1,&PX4StateReporter::px4IMUFilteredCallback,this);
        px4_velocity_estimation_sub = pNH->subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body",1,&PX4StateReporter::px4VelocityEstimationCallback,this);
        px4_gps_satellites_count_sub = pNH->subscribe<std_msgs::UInt32>("/mavros/global_position/raw/satellites",1,&PX4StateReporter::px4GPSSatellitesCountCallback,this);

        pFCStatePub = std::shared_ptr<ros::Publisher>(new ros::Publisher);
        *pFCStatePub = pNH->advertise<gaas_msgs::GAASSystemManagementFlightControllerState>("/gaas/system_management/flight_controller_state",1);
    }
    void initPX4StateReporterNode(int argc,char** argv)
    {
        initROSAndAllSubscribers(argc,argv);
        waitForAll();
        ros::spin();
    }
};


int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("px4_state_reporter_node");
    PX4StateReporter px4sr;
    px4sr.initPX4StateReporterNode(argc,argv);

    return 0;
}
