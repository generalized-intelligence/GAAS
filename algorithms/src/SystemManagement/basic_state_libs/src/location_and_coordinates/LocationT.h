#ifndef GAAS_LOCATION_T_HEADER
#define GAAS_LOCATION_T_HEADER

struct LocationWGS84;
struct LocationWGS84
{
    double lon,lat,alt;
    void serialize();
    void deserialize();
};
typedef LocationWGS84 LocationT;

#endif
