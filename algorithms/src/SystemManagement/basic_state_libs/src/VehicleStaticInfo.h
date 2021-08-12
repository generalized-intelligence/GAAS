#ifndef GAAS_VEHICLE_STATIC_INFO_HEADER
#define GAAS_VEHICLE_STATIC_INFO_HEADER
#include "../../json_request_response_lib/src/JSONSerializableAbstract.h"



class VehicleStaticInfo;
class VehicleStaticInfo:public JSONSerializableAbstract
{
public:
    string vehicle_name;//飞行器名称
    double vehicle_size_x,vehicle_size_y,vehicle_size_z;// 尺寸
    bool isVTOL = true;//支持垂直起降.
    double left_span_hour = 8.0;//剩余续航时间(hour) 默认8h.

    void serialize();
    void deserialize();
    void loadFromConfigFile();//从配置文件加载.
    json getJSONObject();
};
#endif
