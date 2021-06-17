#ifndef BAROMETER_MANAGER_H
#define BAROMETER_MANAGER_H
#include <cmath>


class BarometerManager
{
public:
    BarometerManager();
    const double diff_kpa_per_meter = 0.0127;
    bool init_iterate(double input_baro_val_kpa);
    double get_current_baro_height(double baro_val_in_kpa,bool& valid_out);
    double height_diff_to_init(double baro_val_in,double gps_height_in);
    void set_gps_to_baro_height_transformation(double gps_height,double baro_val,bool& success_out);

    inline double get_gps_minus_baro_height(bool& valid)
    {
        valid = (this->gps_diff_ever_init&&this->ever_init);
        return this->gps_minus_baro_height;
    }
    inline bool get_gps_diff_ever_init()
    {
        return this->gps_diff_ever_init;
    }
    inline bool get_ever_init()
    {
        return this->ever_init;
    }

private:
    vector<double> init_baro_val;
    const int init_val_len = 20;
    const double barometer_variance = 1.0;//in meter.
    double init_baro_val_kpa = -1; //认为这个气压点对应高度为0.
    double gps_minus_baro_height = -1;

    bool ever_init = false;
    bool gps_diff_ever_init = false;
};
BarometerManager::BarometerManager()
{
    ;
}
double BarometerManager::get_current_baro_height(double baro_val_in_kpa,bool& valid_out)
{
    valid_out = this->ever_init;
    return (this->init_baro_val_kpa - baro_val_in_kpa)/diff_kpa_per_meter;
}



bool BarometerManager::init_iterate(double input_baro_val_kpa)
//输入单位规定为kpa.
{
    if(this->ever_init)
    {
        return true;
    }
    if(this->init_baro_val.size()<this->init_val_len)
    {
        this->init_baro_val.push_back(input_baro_val_kpa);
    }
    else
    {
        double sum_val = 0;
        for(int i = 0;i<init_baro_val.size();i++)
        {
            sum_val+= this->init_baro_val[i];
        }
        sum_val/= this->init_baro_val.size();
        this->init_baro_val_kpa = sum_val;
        this->ever_init = true;
        return true;
    }
}
void BarometerManager::set_gps_to_baro_height_transformation(double baro_val_in,double gps_height_in,bool& success_out)
{
    bool valid = false;
    double baro_height = this->get_current_baro_height(baro_val_in,valid);//以初始点为0气压计高度.
    if (!valid)
    {
        success_out = false;
        return;
    }
    this->gps_minus_baro_height = gps_height_in - baro_height;
    gps_diff_ever_init = true;
    success_out = true;
}


#endif
