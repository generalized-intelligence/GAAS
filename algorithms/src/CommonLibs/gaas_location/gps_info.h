#ifndef GAAS_LOCATION_GPS_INFO_H
#define GAAS_LOCATION_GPS_INFO_H




struct GPSInfo
{
    double longitude;
    double latitude;
    double altitude;
    string coordinate_mode;

    const double earth_radius_m = 6371393;
    const double pi_ = 3.1415926535;
    void getRelativeXYZFromLonLatAltInNWUCoordinate(double lon,double lat,double alt,double& x,double& y,double& z)
    {
        x = ((lat-latitude)*pi_/180.0)*earth_radius_m;
        y = ((longitude-lon)*pi_/180.0)*cos(latitude*pi_/180)*earth_radius_m;
        z = alt-altitude;
    }
};

#endif
