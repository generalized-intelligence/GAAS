#include <memory>
#include "utils.h"

//#include "../../flight_stage_manager/src/FlightStageManager.h"
//#include "../../basic_state_libs/src/flight_controller_state.h"
#include <ros/ros.h>
#include <ros/time.h>


#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

#include "gaas_msgs/GAASPerceptionObstacleClustersList.h"
#include "gaas_msgs/GAASNavigationDynamicBlockMap.h"
#include "gaas_msgs/GAASNavigationPath.h"
#include "gaas_msgs/GAASSystemManagementFlightControllerState.h"


//各模块专用检验逻辑
#include "SurveillanceModuleAbstract.h"
#include "PerceptionSurveillance.h"
#include "LocalizationSurveillance.h"
#include "NavigationSurveillance.h"
#include "FlightControllerSurveillance.h"


class StateSurveillanceManager
{
private:
    std::shared_ptr<ros::NodeHandle> pNH = nullptr;
    ros::Subscriber flightControllerStateSubscriber,localizationNDTStatusSubscriber,perceptionStatusSubscriber,mavrosOffboardCommandSubscriber;
    std::shared_ptr<SurveillanceModuleAbstract> flightControllerStateModule,localizationStateModule,perceptionStateModule,mavrosOffboardCommandStateModule;
    void initAllSubscribersAndSurveillanceModules()
    {
        flightControllerStateSubscriber = pNH->subscribe<gaas_msgs::GAASSystemManagementFlightControllerState>
                ("/gaas/system_management/flight_controller_state",1,&StateSurveillanceManager::flightControllerStatusCallback,this);
        localizationNDTStatusSubscriber = pNH->subscribe<geometry_msgs::PoseStamped>
                ("/gaas/localization/registration_pose",1,&StateSurveillanceManager::localizationNDTStatusCallback,this);
        perceptionStatusSubscriber = pNH->subscribe<gaas_msgs::GAASPerceptionObstacleClustersList>
                ("/gaas/perception/euclidean_original_clusters_list",1,&StateSurveillanceManager::perceptionStatusCallback,this);
        mavrosOffboardCommandSubscriber = pNH->subscribe<mavros_msgs::PositionTarget>
                ("/mavros/setpoint_raw/local",1,&StateSurveillanceManager::mavrosOffboardCommandCallback,this);

        flightControllerStateModule = std::dynamic_pointer_cast<SurveillanceModuleAbstract>(std::make_shared<FlightControllerModule>());
        flightControllerStateModule->initSurveillanceModule();
        localizationStateModule = std::dynamic_pointer_cast<SurveillanceModuleAbstract>(std::make_shared<LocalizationSurveillanceModule>());
        localizationStateModule->initSurveillanceModule();
        perceptionStateModule = std::dynamic_pointer_cast<SurveillanceModuleAbstract>(std::make_shared<PerceptionSurveillanceModule>());
        perceptionStateModule->initSurveillanceModule();


    }
public:
    //定义监控其他模块的callback
    //Surveillance callbacks of other modules' status.

    void initStateSurveillanceManagerNode(int argc,char** argv)
    {
        ros::init(argc,argv,"state_surveillance_node");
        pNH = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
        initAllSubscribersAndSurveillanceModules();
        loop();
    };
    void loop()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            onTimerCheckCurrentStatus();
            usleep(10000);//10ms.
        }
    }

    void onTimerCheckCurrentStatus()//计时器触发检查状态. 主要是检查频率和延迟,更新时间等信息.
    {
        ros::Time time_now = ros::Time::now();
        const double TIMEOUT_THRES = 0.5;
        double time_cost_fc = (time_now - this->t_last_fc).toSec();
        if(time_cost_fc > TIMEOUT_THRES&&!t_last_fc.isZero())
        {
            LOG(ERROR)<<"Flight Controller State msg timed out! "<<time_cost_fc<<"sec."<<endl;
        }
        else if(t_last_fc.isZero())
        {
            LOG(WARNING)<<"FCState surveillance still initializing!"<<endl;
        }
        double time_cost_localization = (time_now  - this->t_last_ndt_localization).toSec();
        if(time_cost_localization > TIMEOUT_THRES&&!t_last_ndt_localization.isZero())
        {
            LOG(ERROR)<<"Registration localization msg timed out! "<<time_cost_localization<<"sec."<<endl;
        }
        else if(t_last_ndt_localization.isZero())
        {
            LOG(WARNING)<<"Registration pose still initializing!"<<endl;
        }
        double time_cost_perception = (time_now - this->t_last_perception).toSec();
        if(time_cost_perception > TIMEOUT_THRES&&!t_last_perception.isZero())
        {
            LOG(ERROR)<<"Perception Obstacle Clusters timed out! "<<time_cost_perception<<"sec."<<endl;
        }
        else if(t_last_perception.isZero())
        {
            LOG(WARNING)<<"Perception clusters still initializing!"<<endl;
        }

    }


    void flightControllerStatusCallback(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& pFCStatus); //飞控相关数据汇总.这里定义一个新消息类型兼容不同的飞控.
    //飞控的消息主要是gps坐标和gps质量,电池状态,气压,速度估计等.
    void localizationNDTStatusCallback(const geometry_msgs::PoseStampedConstPtr& pNDTPose);//定位模块状态
    void perceptionStatusCallback(const gaas_msgs::GAASPerceptionObstacleClustersListConstPtr& pPerceptionObsList);//感知模块状态
    //void navigatorPathStatusCallback();//导航模块状态//TODO:加一个类似看门狗的逻辑
    void mavrosOffboardCommandCallback(const mavros_msgs::PositionTargetConstPtr& pCommand);//px4飞控命令 如果不使用px4-mavros就没有这个回调函数
private:
    ros::Time t_last_fc,t_last_ndt_localization,t_last_perception,t_last_navigation_path,t_last_mavros_offboard_command;//更新时间检查.
    bool fc_correct = false,localization_correct = false,perception_correct=false,navigation_correct=false,mavros_offboard_command_correct=false;//内部值域检查.

};
void StateSurveillanceManager::flightControllerStatusCallback(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& pFCStatus)
{
    t_last_fc = pFCStatus->header.stamp;
}
void StateSurveillanceManager::localizationNDTStatusCallback(const geometry_msgs::PoseStampedConstPtr& pNDTPose)
{
    t_last_ndt_localization = pNDTPose->header.stamp;
    auto pLocalizationModule = std::dynamic_pointer_cast<LocalizationSurveillanceModule>(localizationStateModule);
    pLocalizationModule->runPipelineAndGetState(pNDTPose);
}
void StateSurveillanceManager::perceptionStatusCallback(const gaas_msgs::GAASPerceptionObstacleClustersListConstPtr& pPerceptionObsList)
{
    t_last_perception = pPerceptionObsList->header.stamp;
    auto pPerceptionModule = std::dynamic_pointer_cast<PerceptionSurveillanceModule>(perceptionStateModule);
    pPerceptionModule->runPipelineAndGetState(pPerceptionObsList);
}
//void StateSurveillanceManager::navigatorPathStatusCallback(const gaas_msgs::GAASNavigationPathConstPtr& pPath)
//{
//    t_last_navigation_path = pPath->header.stamp;
//}
void StateSurveillanceManager::mavrosOffboardCommandCallback(const mavros_msgs::PositionTargetConstPtr& pCommand)
{
    t_last_mavros_offboard_command = pCommand->header.stamp;
}


int main(int argc,char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("state_surveillance_node");
    //usleep(1000000);//wait 1s.
    StateSurveillanceManager ssm;
    ssm.initStateSurveillanceManagerNode(argc,argv);

    return 0;
}
