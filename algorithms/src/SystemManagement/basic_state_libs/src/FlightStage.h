#ifndef GAAS_FLIGHT_STAGE_HEADER
#define GAAS_FLIGHT_STAGE_HEADER


#include "typedefs.h"

namespace GAASManagement {

typedef unsigned int FlightStageT;
namespace FlightStage{//飞行阶段
    static const FlightStageT NOT_INITIALIZED_STAGE = 1;  //not initialized, unknown state.
    static const FlightStageT GPS_AHRS_STAGE = 2;
    static const FlightStageT LIDAR_LOCALIZATION_STAGE = 4;

    static const FlightStageT STOPPED = 8;
    static const FlightStageT TAKE_OFF_STAGE = 16;
    static const FlightStageT CRUISE_STAGE = 32;
    static const FlightStageT LANDING_STAGE = 64;
}
typedef unsigned int FlightStageT;

}


#endif
