#ifndef SCENE_FRAME_PROPERTIES_H
#define SCENE_FRAME_PROPERTIES_H

class SceneFrame_Properties
{
public:
    SceneFrame_Properties();
    bool queryGPS(double &longi,double &lati);
    bool queryHeading(double &heading);
    void initGPS(double longi,double lati);
    void initHeading(double heading);
    static bool checkGPSDistanceInsideOfRange(double dist_thres_meter,double longi1,double lati1,double longi2,double lati2);
private:
    bool property_gps_info_valid;
    double longitude_approximate;
    double latitude_approximate;
    bool property_magnet_heading_valid;
    double magnet_heading;
    
};
SceneFrame_Properties::SceneFrame_Properties()
{
    this->property_gps_info_valid = false;
    this->longitude_approximate = 0;
    this->latitude_approximate = 0;
    
    this->property_magnet_heading_valid = false;
    this->magnet_heading = 0;
}
bool SceneFrame_Properties::queryGPS(double &longi,double &lati)
{
    if (this->property_gps_info_valid)
    {
        longi = this->longitude_approximate;
        lati = this->latitude_approximate;
        return true;
    }
    return false;
}
bool SceneFrame_Properties::queryHeading(double &heading)
{
    if(this->property_magnet_heading_valid)
    {
        heading = this->magnet_heading;
        return true;
    }
    return false;
}
void SceneFrame_Properties::initGPS(double longi,double lati)//just a approximate estimation of gps,so using gps info inside exif will be ok.
{
    this->property_gps_info_valid = true;
    this->longitude_approximate = longi;
    this->latitude_approximate = lati;
}
void SceneFrame_Properties::initHeading(double heading)
{
    this->property_magnet_heading_valid = true;
    this->magnet_heading = heading;
}


#endif
