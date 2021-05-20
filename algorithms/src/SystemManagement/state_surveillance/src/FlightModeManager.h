#ifndef GAAS_FLIGHT_MODE_MANAGER_HEADER
#define GAAS_FLIGHT_MODE_MANAGER_HEADER
#include "../../basic_state_libs/src/FlightMode.h"
#include "../../basic_state_libs/src/FlightStatus.h"
#include "../../basic_state_libs/src/MapManager.h"
#include "../../basic_state_libs/src/VehicleStaticInfo.h"

namespace GAASManagement {
class GAASFlightModeManager
{
public:
    GAASFlightStatusT current_flight_status;
    MapManager map_manager;
    VehicleStaticInfo vehicle_static_info;

    FlightModeT getCurrentFlightMode();//获取当前状态
    bool reportStatusToGroundControlStationAsync(int report_request_id);//异步向地面站报告状态.
    int queryReportRequestStatus(int report_request_id);//查询是否报告完成.
    bool tryFlightModeStateTransfer(const FlightModeT& new_mode);//尝试状态转换

    //定义监控其他模块的callback
    //Surveillance callbacks of other modules' status.
    void flightControllerStatusCallback(); //飞控相关数据汇总.这里定义一个新消息类型兼容不同的飞控.
    //飞控的消息主要是gps坐标和gps质量,电池状态,气压,速度估计等.
    void localizationStatusCallback();//定位模块状态
    void perceptionStatusCallback();//感知模块状态
    void navigatorStatusCallback();//导航模块状态

};

}

#endif
