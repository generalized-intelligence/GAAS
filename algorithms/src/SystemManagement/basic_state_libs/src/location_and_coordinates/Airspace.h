#ifndef GAAS_AIRSPACE_HEADER
#define GAAS_AIRSPACE_HEADER

#include "LocationT.h"

struct AirspaceRange; // 空域范围
struct AirspaceStatus;//空域当前状态.


struct AirspaceRange
{
    bool isInRange(const LocationT& current_location);//判断是否在空域内部.
};



struct AirspaceStatus
{
    const static int STATUS_CLEAR_AND_SAFE = 0;
    const static int STATUS_DANGER = 1;

    int space_status = STATUS_CLEAR_AND_SAFE;

    bool support_landing = true;//可以降落
    bool HDmap_avail = true;//有高精地图
    string map_name="unknown_map_name";//地图名称.

    LocationT airspace_center;
    AirspaceRange airspace_range;

    double temperature_celsius;//当地温度

    double windspeed_N_mps;//当地风速测量 m/s
    double windspeed_W_mps;
    double windspeed_U_mps;
    string weather = "Sunny";//默认是晴.

    void serialize();
    void deserialize();
};



#endif
