#ifndef ROS_WRAPPER_MESHGEN
#define ROS_WRAPPER_MESHGEN

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "Pipeline.h"
#include <thread>







class MeshGeneratorROSWrapper
{
private:

public:
    void onCallback(shared_ptr<cvMatT> im1,shared_ptr<cvMatT> im2,shared_ptr<cvMatT> im3,shared_ptr<cvMatT> im4)
    {
        std::thread t1(run_all_pipeline,im1,im2);
        std::thread t2(run_all_pipeline,im3,im4);
        t1.join();
        t2.join();
    }
};



#endif
