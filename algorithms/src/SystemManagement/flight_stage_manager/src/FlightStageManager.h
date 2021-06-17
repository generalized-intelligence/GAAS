#ifndef GAAS_FLIGHT_MODE_MANAGER_HEADER
#define GAAS_FLIGHT_MODE_MANAGER_HEADER
#include "../../basic_state_libs/src/FlightStage.h"
#include "../../basic_state_libs/src/FlightStatus.h"
#include "../../basic_state_libs/src/MapManager.h"
#include "../../basic_state_libs/src/VehicleStaticInfo.h"



namespace GAASManagement {
class GAASFlightStageManager
{
public:
    GAASFlightStatusT current_flight_status;
    MapManager map_manager;
    VehicleStaticInfo vehicle_static_info;

    FlightStageT getCurrentFlightStage();//获取当前阶段
    bool reportStatusToGroundControlStationAsync(int report_request_id);//异步向地面站报告状态.
    int queryReportRequestStatus(int report_request_id);//查询是否报告完成.
    bool tryFlightStageStateTransfer(const FlightStageT& new_mode);//尝试状态转换

    void visualizeCurrentState()
    {
        ;
    }

};

}

#endif
