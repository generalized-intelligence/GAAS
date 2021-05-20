#ifndef GAAS_FLIGHT_STATUS_HEADER
#define GAAS_FLIGHT_STATUS_HEADER

#include "FlightMode.h"
#include "location_and_coordinates/LocationT.h"

namespace GAASManagement {

struct GAASFlightStatusT//用于和地面同步飞行状态.
{
    string current_map_name;
    FlightModeT current_mode = FlightMode::IDLE_STATE;
    LocationT currentLocation,TargetLocation;


    void initFlightStatus(const string& curr_map_name,const FlightModeT& curr_mode,
                          const LocationT& curr_location,const LocationT& target_location);
    LocationT getCurrentLocation();
    LocationT getTargetLocation();
    void serialize();
    void deserialize();
};


}

#endif
