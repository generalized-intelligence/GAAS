#ifndef GPS_UTILS_H
#define GPS_UTILS_H
#include <cmath>
namespace GPS_Utils
{
    const double earth_radius_km = 6371.393;
    const double pi_ = 3.1415926535;
    void get_longitude_range_by_dist(double distance_range_km,double center_lati,double &output_range_longitude,bool &success)
    {
        if(center_lati<90.0 && center_lati>-90.0)
        {
            output_range_longitude =(180/(2*pi_))* 
				(distance_range_km / 
				earth_radius_km * cos(
							(abs(center_lati)*2*pi_)/180.0
						)
				);
            success = true;
        }
        else
        {
            success = false;
        }
    }



}




#endif
