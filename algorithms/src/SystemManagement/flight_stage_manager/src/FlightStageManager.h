#ifndef GAAS_FLIGHT_MODE_MANAGER_HEADER
#define GAAS_FLIGHT_MODE_MANAGER_HEADER
#include "../../basic_state_libs/src/FlightStage.h"
#include "../../basic_state_libs/src/FlightStatus.h"
#include "../../basic_state_libs/src/VehicleStaticInfo.h"
#include "../../../CommonLibs/gaas_map/registration_map_manager.h"
#include <opencv2/core/persistence.hpp>


namespace GAASManagement {
//We need a standard for flight missions of GAAS. So a domain specific language is needed.


class GAASFlightStageManager;
class FlightMission
{
public:
    using Ptr = std::shared_ptr<FlightMission>;
    std::string type=std::string("unknown");
    virtual ~FlightMission()
    {
        ;
    }

};
class MissionTakeoff: public FlightMission
{
public:
    using Ptr = std::shared_ptr<MissionTakeoff>;
    double height_m;
};
class MissionAccuratePath: public FlightMission
{
public:
    using Ptr = std::shared_ptr<MissionAccuratePath>;
    typedef std::array<double,3> PathPoint3dT;
    std::vector<PathPoint3dT> points;
    std::string map_name;

};
class MissionCruise: public FlightMission
{
public:
    using Ptr = std::shared_ptr<MissionCruise>;
    typedef std::array<double,3> LonLatAltT;
    std::vector<LonLatAltT> points;

};


class FlightMissionParser
{
protected:
    //parse params.
    void parseTakeoff(const cv::FileNodeIterator& it, MissionTakeoff::Ptr& pMission);
    void parseMapAccuratePath(const cv::FileNodeIterator& it, MissionAccuratePath::Ptr& pMission);
    void parseCruise(const cv::FileNodeIterator& it, MissionCruise::Ptr& pMission);
public:
    //parse types.
    std::vector<FlightMission::Ptr> parseMissions(const std::string& mission_file_path)
    //implementation of a parser to parse a file of FlightMission.
    {
        cv::FileStorage fs;
        fs.open(mission_file_path,cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            LOG(ERROR)<<"ERROR: mission_file_path invalid!"<<endl;
            throw "Error!";
        }
        vector<FlightMission::Ptr> missions_ret;
        auto missions = fs["Missions"];
        cv::FileNodeIterator it = missions.begin(), it_end = missions.end();
        for (; it != it_end; ++it)
        {
            std::string mission_type;
            (*it)["type"]>>mission_type;
            if(mission_type == "takeoff")
            {
                MissionTakeoff::Ptr pmission_takeoff(new MissionTakeoff);
                parseTakeoff(it,pmission_takeoff);
                FlightMission::Ptr pFlightMission = std::dynamic_pointer_cast<FlightMission>(pmission_takeoff);
                missions_ret.push_back(pFlightMission);
            }
            else if(mission_type == "map_accurate_path")
            {
                MissionAccuratePath::Ptr pmission_map_path;
                parseMapAccuratePath(it,pmission_map_path);
                FlightMission::Ptr pFlightMission = std::dynamic_pointer_cast<FlightMission>(pmission_map_path);
                missions_ret.push_back(pFlightMission);
            }
            else if(mission_type == "mission_cruise")
            {
                MissionCruise::Ptr pmission_cruise;
                parseCruise(it,pmission_cruise);
                FlightMission::Ptr pFlightMission = std::dynamic_pointer_cast<FlightMission>(pmission_cruise);
                missions_ret.push_back(pFlightMission);
            }
            else
            {
                LOG(ERROR)<<"Unknown mission type, check your mission file!"<<endl;
                throw("error");
            }
        }
        return missions_ret;
    }
};
void FlightMissionParser::parseTakeoff(const cv::FileNodeIterator &it, MissionTakeoff::Ptr &pMission)
{
    (*it)["height_m"]>>pMission->height_m;
}
void FlightMissionParser::parseCruise(const cv::FileNodeIterator &it, MissionCruise::Ptr &pMission)
{
    auto targets_node = (*it)["targets"];
    cv::FileNodeIterator targets_it = targets_node.begin(), targets_it_end = targets_node.end();
    for(;targets_it!=targets_it_end;++targets_it)
    {
        MissionCruise::LonLatAltT gps_target;
        double lon,lat,alt;
        (*targets_it)["lon"] >> lon;
        (*targets_it)["lat"] >> lat;
        (*targets_it)["alt"] >> alt;
        gps_target.at(0) = lon;
        gps_target.at(1) = lat;
        gps_target.at(2) = alt;
        pMission->points.push_back(gps_target);
    }
}
void FlightMissionParser::parseMapAccuratePath(const cv::FileNodeIterator &it, MissionAccuratePath::Ptr &pMission)
{
    (*it)["map"]>> pMission->map_name; // set map name. the key of map config file.
    auto targets_node = (*it)["targets"];
    cv::FileNodeIterator targets_it = targets_node.begin(), targets_it_end = targets_node.end();
    for(;targets_it!=targets_it_end;++targets_it)
    {
        MissionAccuratePath::PathPoint3dT pt;
        double x,y,z;
        (*targets_it)["x"]>>x;
        (*targets_it)["y"]>>y;
        (*targets_it)["z"]>>z;
        pt.at(0) = x;
        pt.at(1) = y;
        pt.at(2) = z;
        pMission->points.push_back(pt);
    }
}





class GAASFlightStageManager
{
protected:
    FlightStageT flight_stage = FlightStage::STOPPED|FlightStage::GPS_AHRS_STAGE|FlightStage::NOT_INITIALIZED_STAGE;

    RegistrationMapManager::Ptr map_manager;
    bool initFlightStageManager(std::shared_ptr<ros::NodeHandle> pNH);

    //localization switch
    bool switchToGPS_AHRSLocalization();
    bool switchToLidarLocalization();



    //state tranfer
    bool switchToTakeoffMode();
    bool switchToCruiseMode();
    bool switchToLandMode();
    bool doTakeoffMission(MissionTakeoff::Ptr pMission)
    {
        ;
    }
    bool doMapAccuratePath(MissionAccuratePath::Ptr pMission)
    {
        map_manager->selectMapByName(pMission->map_name);
        for(const auto& pt:pMission->points)
        {
            //if(!map_manager->check_coordinate_valid(pt))//TODO:检查地图坐标合法性.
            //{
            //    LOG(ERROR)<<"Invalid coordinate in map:"<<map_name<<". check coordinate:"<<pt[0]<<","<<pt[1]<<","<<pt[2]<<endl;
            //    throw "error!";
            //}
        }
        for(const auto& pt:pMission->points)
        {
            //设置目标并检查是否完成
            //auto submission = setMapAccurateTarget(map_name,pt);
            //submission.waitTillFinished();
        }
    }
    bool doCruise(MissionCruise::Ptr pMission)
    {
        for(const auto pt:pMission->points)
        {
            //设置并检查目标完成
            //auto submission = setGlobalGPSTarget(pt);
            //submission.waitTillFinished();
        }
    }

public:
    bool launchFlightStageManagerWithMissionFile(const string& file_path)//execute mission file.
    {
        FlightMissionParser parser;
        vector<FlightMission::Ptr> missions = parser.parseMissions(file_path);
        for(const FlightMission::Ptr& mission_ptr:missions)
        {
            tryFlightStageStateTransfer(mission_ptr->type); //状态转移
            //do operations.
            if(mission_ptr->type == "takeoff")
            {
                auto pMission = std::dynamic_pointer_cast<MissionTakeoff>(mission_ptr);
                doTakeoffMission(pMission);
            }
            else if(mission_ptr->type == "map_accurate_path")
            {
                auto pMission = std::dynamic_pointer_cast<MissionAccuratePath>(mission_ptr);
                doMapAccuratePath(pMission);
            }
            else if(mission_ptr->type == "mission_cruise")
            {
                auto pMission = std::dynamic_pointer_cast<MissionCruise>(mission_ptr);
                doCruise(pMission);
            }
        }
    }



    FlightStageT getCurrentFlightStage();//获取当前阶段
    bool reportStatusToGroundControlStationAsync(int report_request_id);//异步向地面站报告状态.
    int queryReportRequestStatus(int report_request_id);//查询是否报告完成.
    bool tryFlightStageStateTransfer(const std::string& state);//尝试状态转换 包含检查之前任务完成状态 处理状态机等操作
    void setMapAccurateTarget(const std::string& map_name,const MissionAccuratePath::PathPoint3dT& target);
    void visualizeCurrentState()
    {
        ;
    }

};

}

#endif
