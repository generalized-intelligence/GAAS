#include "../../flight_stage_manager/src/FlightStageManager.h"


#include "../../basic_state_libs/src/flight_controller_state.h"

class StateSurveillanceManager
{
public:
    //定义监控其他模块的callback
    //Surveillance callbacks of other modules' status.
    void flightControllerStatusCallback(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& pFCStatus); //飞控相关数据汇总.这里定义一个新消息类型兼容不同的飞控.
    //飞控的消息主要是gps坐标和gps质量,电池状态,气压,速度估计等.
    void localizationStatusCallback();//定位模块状态
    void perceptionStatusCallback();//感知模块状态
    void navigatorStatusCallback();//导航模块状态
};

int main(int argc,char** argv)
{
    return 0;
}
