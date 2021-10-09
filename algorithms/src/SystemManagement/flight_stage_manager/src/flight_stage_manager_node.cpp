#include "FlightStageManager.h"

int main(int argc, char** argv)
{
    GAASManagement::FlightMissionParser fmp;
    fmp.parseMissions("/home/gi/GAAS/config/gaas_mission_demo.yaml");
    return 0;
}
