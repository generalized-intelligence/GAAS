#ifndef GAAS_FLIGHT_MODE_HEADER
#define GAAS_FLIGHT_MODE_HEADER


#include "typedefs.h"

namespace GAASManagement {

typedef unsigned int FlightModeT;
namespace FlightMode{

    static const FlightModeT IDLE_STATE = 1;
    static const FlightModeT TAKEOFF_STATE = 2;
    static const FlightModeT CRUISE_STATE = 4;
    static const FlightModeT LANDING_STATE = 8;
}
typedef unsigned int FlightModeT;

}


#endif
