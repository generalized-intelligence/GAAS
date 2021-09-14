#ifndef GPS_AHRS_SYNC_HEADER_FILE
#define GPS_AHRS_SYNC_HEADER_FILE


#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <mutex>

struct GPS_AHRS_Synchronizer
{
    using Ptr = std::shared_ptr<GPS_AHRS_Synchronizer>;
    nav_msgs::Odometry ahrs_msg;
    sensor_msgs::NavSatFix gps_msg;
    bool ever_init = false;
    std::mutex sync_mutex;

    bool get_sync_gps_ahrs_msgs(sensor_msgs::NavSatFix& gps,nav_msgs::Odometry& ahrs)
    {
        sync_mutex.lock();
        if(!ever_init)
        {
            sync_mutex.unlock();
            return false;
        }
        else
        {
            gps = this->gps_msg;
            ahrs = this->ahrs_msg;
            sync_mutex.unlock();
            return true;
        }
    }
};



#endif
