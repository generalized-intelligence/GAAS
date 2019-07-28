
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include "GPSExpand.h"
#include <deque>

class GPSModule
{
public:
    GPSModule(const cv::FileStorage& fs)
    {
        this->fSettings = fs;
    }
    void init_with_msg_buffer(std::vector<sensor_msgs::NavSatFix> history_gps_vec);
    bool check_gps_avail(const VertexPR& lastPos,GPSExpand& gps_expand,double xy_threshold)
    {
        //check if gps_value in 3 sigma.
        //TODO.
    }
    bool iterate(const sensor_msgs::NavSatFix &gps_msg)
    {
        this->msg_queue.push_front(gps_msg);
        if(gps_msg.status.status>=gps_msg.status.STATUS_FIX)
        {
            return true;
        }
        return false;
    }
private:
    deque <sensor_msgs::NavSatFix> msg_queue;
    cv::FileStorage& fSettings;
};

