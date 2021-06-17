#include "CallbacksBufferBlock.h"
#include "GlobalOptimizationGraph.h"
#include "ROS_IO_Manager.h"
#include <memory>
#include <thread>
#include <chrono>
#include <csignal>
#include <unistd.h>
#include <glog/logging.h>
 
void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

bool init(shared_ptr<ROS_IO_Manager> pRIM, shared_ptr<GlobalOptimizationGraph> pGOG,
            int argc,char** argv)
{
    //to start global optimization,first check all input topics.
    cv::FileStorage fSettings;
    fSettings.open(string(argv[1]),cv::FileStorage::READ);
    
    pRIM->setOptimizationGraph(pGOG);

    return pRIM->initOptimizationGraph();
}

void loop(shared_ptr<ROS_IO_Manager> pRIM, shared_ptr<GlobalOptimizationGraph> pGOG)
{
    int count = 0;
    while(ros::ok())
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
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging(argv[0]);

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
