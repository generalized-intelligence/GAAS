#ifndef PERCEPTION_SURVEILLANCE
#define PERCEPTION_SURVEILLANCE
#include "SurveillanceModuleAbstract.h"
#include "gaas_msgs/GAASPerceptionObstacleClustersList.h"
#include <sstream>

class PerceptionSurveillanceModule:public SurveillanceModuleAbstract
{
public:
    int runPipelineAndGetState(const gaas_msgs::GAASPerceptionObstacleClustersListConstPtr& pPerceptionObsList)
    {
        if(pPerceptionObsList->obstacles.size()>0)
        {
            bool flag_not_empty = false;
            for(const auto& obj:pPerceptionObsList->obstacles)
            {
                if(obj.cloud.width>0)
                {
                    flag_not_empty = true;
                }
            }
            //check msg and set state.
            if(flag_not_empty)
            {
                this->current_status = this->STATUS_CORRECT;
            }
            else
            {
                this->current_status = this->STATUS_NOT_SURE;
            }
        }
        else
        {
            this->current_status = this->STATUS_NOT_SURE;
        }
        std::stringstream ss;
        ss<<"Obstacles count:"<<pPerceptionObsList->obstacles.size();
        std::string log_string = ss.str();
        this->SURVEILLANCE_LOG_STATUS(log_string);
        return checkStateLegal();
    }
    virtual void initSurveillanceModule()
    {
        setModuleName("Perception Module");
    }
    virtual int checkStateLegal()
    {//无上下文依赖.直接返回.
        return this->current_status;
    }
private:


};



#endif
