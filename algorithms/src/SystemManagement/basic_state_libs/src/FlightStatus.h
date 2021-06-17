#ifndef GAAS_FLIGHT_STATUS_HEADER
#define GAAS_FLIGHT_STATUS_HEADER

#include "FlightStage.h"
#include "location_and_coordinates/LocationT.h"

namespace GAASManagement {

struct GAASFlightStatusT//用于和地面同步飞行状态.
{
    string current_map_name;
    FlightStageT current_mode = FlightStage::IDLE_STAGE;
    LocationT currentLocation,TargetLocation;


    void initFlightStatus(const string& curr_map_name,const FlightStageT& curr_stage,
                          const LocationT& curr_location,const LocationT& target_location);
    LocationT getCurrentLocation();
    LocationT getTargetLocation();
    void serialize();
    void deserialize();
};


}

#endif
