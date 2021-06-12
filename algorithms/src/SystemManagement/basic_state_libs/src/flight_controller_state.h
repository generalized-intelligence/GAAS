#ifndef GAAS_FLIGHT_CONTROLER_HEADER
#define GAAS_FLIGHT_CONTROLER_HEADER

#include "gaas_msgs/GAASSystemManagementFlightControllerState.h"
#include <map>
#include <string>
#include <sstream>

#include "../../json_request_response_lib/src/third_party/nlohmann_json/single_include/nlohmann/json.hpp"


using std::string;
using json = nlohmann::json;

struct FlightControllerState // 通用的飞控状态 不局限于某一种.
{
    //定位模式
    const static int GPS_GUIDED_FLIGHT_CONTROLLER = 1;
    const static int LIDAR_GUIDED_FLIGHT_CONTROLLER = 2;
    const static int OPT_FLOW_GUIDED_FLIGHT_CONTROLLER = 4;

    //飞行模式.
    const static int UNKNOWN_MODE = 0;
    const static int OFFBOARD_MODE = 1;
    const static int POSCTL_MODE = 2;
    const static int FLIGHCONTROLLER_AUTO_MODE = 4; // 飞控自动模式,不受GAAS控制.
    const static int ALTITUDE_MODE = 8;
    const static int STABILIZE_MODE = 16;
    const static int MANUAL_MODE = 32;

    const static int STATUS_NO_FIX=-1;
    const static int STATUS_FIX=0;
    const static int STATUS_SBAS_FIX=1;
    const static int STATUS_GBAS_FIX=2;


    //About mavros state:
    bool fc_connected = false; // 是否已经连接飞控.
    int flight_mode = UNKNOWN_MODE;
    bool armed = false;

    //mavros battery:
    double battery_soc_percentage;

    //mavros imu:
    double temperature_celsius;
    double imu_ax,imu_ay,imu_az,imu_wx,imu_wy,imu_wz,imu_filtered_qx,imu_filtered_qy,imu_filtered_qz,imu_filtered_qw;//acc gyro quaternion.

    //guide mode:
    int guide_mode = GPS_GUIDED_FLIGHT_CONTROLLER;

    //About gps:
    int gps_avail_satellite_count;


    int gps_fix_status = STATUS_NO_FIX;
    double gps_longitude,gps_latitude,gps_altitude;
    double gps_lon_dop;
    double gps_lat_dop;
    double gps_alt_dop;
    //About vehicle velocity:
    double vehicle_vx,vehicle_vy,vehicle_vz;

    std::map<string,string> optional_info;//其他可选内容.
    void serialize(); // to json
    void deserialize(); // from json
    void toROSMsg(gaas_msgs::GAASSystemManagementFlightControllerState& msg_);
    void fromROSMsg(const gaas_msgs::GAASSystemManagementFlightControllerState& msg_);
};
void FlightControllerState::toROSMsg(gaas_msgs::GAASSystemManagementFlightControllerState &msg_)
{
    msg_.fc_connected = this->fc_connected;
    msg_.flight_mode = this->flight_mode;
    msg_.armed = this->armed;
    msg_.battery_soc_percentage = this->battery_soc_percentage;
    msg_.temperature_celsius = this->temperature_celsius;
    msg_.imu_ax = imu_ax;
    msg_.imu_ay = imu_ay;
    msg_.imu_az = imu_az;

    msg_.imu_wx = imu_wx;
    msg_.imu_wy = imu_wy;
    msg_.imu_wz = imu_wz;

    msg_.imu_filtered_qx = imu_filtered_qx;
    msg_.imu_filtered_qy = imu_filtered_qy;
    msg_.imu_filtered_qz = imu_filtered_qz;
    msg_.imu_filtered_qw = imu_filtered_qw;

    msg_.guide_mode = guide_mode;
    msg_.gps_avail_satellite_count = gps_avail_satellite_count;
    msg_.gps_fix_status = gps_fix_status;
    msg_.gps_longitude = gps_longitude;
    msg_.gps_latitude = gps_latitude;
    msg_.gps_altitude = gps_altitude;

    msg_.gps_lon_dop = gps_lon_dop;
    msg_.gps_lat_dop = gps_lat_dop;
    msg_.gps_alt_dop = gps_alt_dop;

    msg_.vehicle_vx = vehicle_vx;
    msg_.vehicle_vy = vehicle_vy;
    msg_.vehicle_vz = vehicle_vz;

    //msg_.armed.data
    //可选字段序列化.
    json j;
    for(std::map<string,string>::iterator kv_iter=this->optional_info.begin();kv_iter!=this->optional_info.end();++kv_iter)
    {
        j[kv_iter->first] = kv_iter->second;
    }
    msg_.optinal_values_json.data = j.dump(4);  //indent=4 for sanity.
}
void FlightControllerState::fromROSMsg(const gaas_msgs::GAASSystemManagementFlightControllerState &msg_)
{
    this->fc_connected = msg_.fc_connected;
    this->flight_mode = msg_.flight_mode;
    this->armed = msg_.armed;
    this->battery_soc_percentage =  msg_.battery_soc_percentage;
    this->temperature_celsius = msg_.temperature_celsius;
    imu_ax = msg_.imu_ax;
    imu_ay = msg_.imu_ay;
    imu_az = msg_.imu_az;

    imu_wx = msg_.imu_wx;
    imu_wy = msg_.imu_wy;
    imu_wz = msg_.imu_wz;

    imu_filtered_qx = msg_.imu_filtered_qx;
    imu_filtered_qy = msg_.imu_filtered_qy;
    imu_filtered_qz = msg_.imu_filtered_qz;
    imu_filtered_qw = msg_.imu_filtered_qw;

    guide_mode = msg_.guide_mode;
    gps_avail_satellite_count = msg_.gps_avail_satellite_count;
    gps_fix_status = msg_.gps_fix_status;
    gps_longitude = msg_.gps_longitude;
    gps_latitude = msg_.gps_latitude;
    gps_altitude = msg_.gps_altitude;

    gps_lon_dop = msg_.gps_lon_dop;
    gps_lat_dop = msg_.gps_lat_dop;
    gps_alt_dop = msg_.gps_alt_dop;

    vehicle_vx = msg_.vehicle_vx;
    vehicle_vy = msg_.vehicle_vy;
    vehicle_vz = msg_.vehicle_vz;

    //msg_.armed.data
    //可选字段序列化.
    this->optional_info.clear();
    json j;
    std::stringstream ss;
    ss<<msg_.optinal_values_json.data;
    ss>>j;
    for (json::iterator it = j.begin(); it != j.end(); ++it)
    {
        this->optional_info[it.key()] = it.value();
    }
    return;
}
#endif
