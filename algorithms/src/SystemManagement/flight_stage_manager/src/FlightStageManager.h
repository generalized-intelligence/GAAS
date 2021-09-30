#ifndef GAAS_FLIGHT_MODE_MANAGER_HEADER
#define GAAS_FLIGHT_MODE_MANAGER_HEADER
#include "../../basic_state_libs/src/FlightStage.h"
#include "../../basic_state_libs/src/FlightStatus.h"
#include "../../basic_state_libs/src/VehicleStaticInfo.h"
#include "../../../CommonLibs/gaas_map/registration_map_manager.h"



namespace GAASManagement {
//We need a standard for flight missions of GAAS. So a domain specific language is needed.

typedef int FlightMissionType;
static const FlightMissionType TAKEOFF = 0;

class FlightMission
{
public:
    FlightMissionType type;

};
class MissionTakeoff: public FlightMission
{
    double height_m;
};
class MissionAccuratePath: public FlightMission
{
    typedef std::array<double,3> PathPoint3dT;
    std::vector<PathPoint3dT> points;
};
class MissionCruise: public FlightMission
{
    typedef std::array<double,3> LonLatAltT;
    std::vector<LonLatAltT> points;
};


class FlightMissionParser
{
public:
    void parse(const string& full_text)
    //implementation of a parser to parse a file of FlightMission.
    {
        ;
    }
};


class GAASFlightStageManager
{
private:
    FlightStageT flight_stage = FlightStage::STOPPED|FlightStage::GPS_AHRS_STAGE|FlightStage::NOT_INITIALIZED_STAGE;
public:
    RegistrationMapManager::Ptr map_manager;
    bool initFlightStageManager(std::shared_ptr<ros::NodeHandle> pNH);

//localization switch
    bool switchToGPS_AHRSLocalization();
    bool switchToLidarLocalization();



//state tranfer
    bool switchToTakeoffMode();
    bool switchToCruiseMode();
    bool switchToLandMode();

    bool launchFlightStageManagerWithMissionFile(const string& file_path)//execute mission file.
    {
        ;
    }



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
