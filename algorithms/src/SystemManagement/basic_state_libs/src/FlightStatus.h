#ifndef GAAS_FLIGHT_STATUS_HEADER
#define GAAS_FLIGHT_STATUS_HEADER

#include "FlightStage.h"
#include "location_and_coordinates/LocationT.h"
#include "../../json_request_response_lib/src/JSONSerializableAbstract.h"

namespace GAASManagement {

class GAASFlightStatusT:public JSONSerializableAbstract//用于和地面同步飞行状态.
{
public:
    string current_map_name;
    FlightStageT current_mode = FlightStage::IDLE_STAGE;
    LocationT currentLocation,TargetLocation;


    void initFlightStatus(const string& curr_map_name,const FlightStageT& curr_stage,
                          const LocationT& curr_location,const LocationT& target_location);
    LocationT getCurrentLocation();
    LocationT getTargetLocation();
    void serialize();
    void deserialize();
    json getJSONObject();
};


}

#endif
