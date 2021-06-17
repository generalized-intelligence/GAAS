
#include "CallbacksBufferBlock.h"
#include "GlobalOptimizationGraph.h"
#include "ROS_IO_Manager.h"
#include <memory>
#include <thread>
#include <chrono>





bool init(shared_ptr<ROS_IO_Manager> pRIM,shared_ptr<GlobalOptimizationGraph> pGOG,
            int argc,char** argv)
{
    //to start global optimization,first check all input topics.
    cv::FileStorage fSettings;
    fSettings.open(string(argv[1]),cv::FileStorage::READ);
    
    pRIM->setOptimizationGraph(pGOG);
    time_us_t t1 = micros();
    int init_spin_times = fSettings["INIT_SPIN_TIMES"];
    for(int i=0;i<init_spin_times;i++)
    {
        ros::spinOnce();//store enough msgs to init buffer.
        //ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        //So we do not worry about call block.
        time_us_t current_t = micros();
        int init_max_time_us = fSettings["INIT_MAX_TIME_us"];
        time_us_t init_max_time_us_time = (time_us_t)init_max_time_us;
        if (current_t - t1 > init_max_time_us_time) // ensure this step shall be finished in time.
        {
            cout <<"max time:"<<init_max_time_us_time<<endl;
            cout <<"current time cost:"<<current_t - t1<<endl;
            break;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));//sleep 1ms.
        }
    }
    
    /*
    if (fSettings["ENABLE_GPS"])
    {
        bool gps_valid = pRIM->tryInitGPS();
    }
    if(!pRIM->tryInitSLAM())
    {
        cout<<"SLAM init failed."<<endl;
        return false;
    }*/
    return pRIM->initOptimizationGraph();
    //pRIM->tryInitVelocity();
}

void loop(shared_ptr<ROS_IO_Manager> pRIM,shared_ptr<GlobalOptimizationGraph> pGOG)
{
    int count = 0;
    while(true)
    {
        bool stateUpdated = pRIM->loopFunc();
        if(stateUpdated)
        {
            cout<<"Loop count:"<<count<<",State:"<<stateUpdated<<"."<<endl;
            count++;
        }
        else
        {
            //cout<<"RIM Idling."<<endl;
        }
    }
}


int main(int argc,char** argv)
{
    //step<1> check input.
    cout<<"Usage:GlobalOptimization_main [config_file_path]"<<endl;
    if (argc<2)
    {
        cout<<"Input error!"<<endl;
        return -1;
    }
    //step<2> init ROS_IO_Manager. ROS operation inside;
    shared_ptr<ROS_IO_Manager> pRIM(new ROS_IO_Manager(argc,argv));
    //step<3> init GlobalOptimizationGraph.
    shared_ptr<GlobalOptimizationGraph> pGOG(new GlobalOptimizationGraph(argc,argv));
    //init()
    if (!init(pRIM,pGOG,argc,argv))
    {
        cout<<"Init failed!"<<endl;
        return -1;
    };
    cout<<"Init success,start loop!"<<endl;
    loop(pRIM,pGOG);
    return 0;
}
