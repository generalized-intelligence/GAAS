
#include "CallbacksBufferBlock.h"
#include "GlobalOptimizationGraph.h"
#include "ROS_IO_Manager.h"
#include <memory>
#include <opencv/opencv2.hpp>



bool init(shared_ptr<ROS_IO_Manager> pRIM,shared_ptr<GlobalOptimizationGraph> pGOG,
            int argc,char** argv)
{
    //to start global optimization,first check all input topics.
    cv::FileStorage fSettings(string(argv[1]),cv::FileStorage::READ);
    time_us_t t1 = micros();
    for(int i=0;i<fSettings["INIT_SPIN_TIMES"],i++)
    {
        ros::spinOnce();//store enough msgs to init buffer.
        //ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        //So we do not worry about call block.
        time_us_t current_t = micros();
        if (current_t - t1 > fSettings["INIT_MAX_TIME_us"]) // ensure this step shall be finished in time.
        {
            break;
        }
    }
    pRIM->setOptimizationGraph(pGOG);
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
    while(true)
    {
        bool stateCorrect = pRIM->loopFunc();
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
    shared_ptr<GlobalOptimizationGraph> pGOG(new GlobalOptimizationGraph());
    //init()
    if (!init(pRIM,pGOG,argc,argv))
    {
        cout<<"Init failed!"<<endl;
        return -1;
    };
    loop(pRIM,pGOG);
    return 0;
}
