#ifndef GAAS_FLIGHT_STAGE_HEADER
#define GAAS_FLIGHT_STAGE_HEADER


#include "typedefs.h"

namespace GAASManagement {

typedef unsigned int FlightStageT;
namespace FlightStage{//飞行阶段

    static const FlightStageT IDLE_STAGE = 1;
    static const FlightStageT TAKEOFF_STAGE = 2;
    static const FlightStageT CRUISE_STAGE = 4;
    static const FlightStageT LANDING_STAGE = 8;
}
typedef unsigned int FlightStageT;

}


#endif
