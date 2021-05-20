#ifndef GAAS_FLIGHT_CONTROLER_HEADER
#define GAAS_FLIGHT_CONTROLER_HEADER




struct FlightControllerState // 通用的飞控状态 不局限于某一种.
{
    bool fc_connected = false; // 是否已经连接飞控.
    double battery_soc;
    double temperature_celsius;
    const static int GPS_GUIDED_FLIGHT_CONTROLLER = 1;
    const static int LIDAR_GUIDED_FLIGHT_CONTROLLER = 2;
    const static int OPT_FLOW_GUIDED_FLIGHT_CONTROLLER = 4;

    int guide_mode = GPS_GUIDED_FLIGHT_CONTROLLER;
    bool armed = false;
    int gps_avail_satellite_count;

    void serialize();
    void deserialize();
};


#endif
