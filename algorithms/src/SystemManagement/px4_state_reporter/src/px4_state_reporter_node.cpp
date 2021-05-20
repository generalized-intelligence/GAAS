#include "../../basic_state_libs/src/flight_controller_state.h"
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

class PX4StateReporter
{
public:
    ros::Subscriber px4_state_sub,px4_battery_sub,px4_GPSNavSatFix_sub;
    bool   px4_state_init = false, px4_battery_init = false, px4_GPSNavSatFix_init = false;

    FlightControllerState curr_state;
    FlightControllerState getStateFromPX4State(const mavros_msgs::State& px4_state);
    std::shared_ptr<ros::Publisher> pFCStatePub;

    void px4StateCallback(const mavros_msgs::StateConstPtr& p_px4_state_msg)
    {
        p_px4_state_msg;
    }
    void px4BatteryStateCallback(const mavros_msgs::BatteryStatusConstPtr& p_battery_msg)
    {
        ;
    }
    void px4GPSNavSatFixCallback(const sensor_msgs::NavSatFixConstPtr& p_NavSatFix)
    {
        ;
    }
    void waitForAll()//初始化所有状态
    {
        ;
    }
};


int main(int argc,char** argv)
{
    return 0;
}
