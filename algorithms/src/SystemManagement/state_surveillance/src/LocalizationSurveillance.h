#ifndef LOCALIZATION_SURVEILLANCE
#define LOCALIZATION_SURVEILLANCE
#include "SurveillanceModuleAbstract.h"
#include <geometry_msgs/PoseStamped.h>

class LocalizationSurveillanceModule:SurveillanceModuleAbstract
{
    int runPipelineAndGetState(const geometry_msgs::PoseStampedConstPtr& pRegistrationPose)
    {
        if(!this->ever_init)
        {
            this->lastPose= *pRegistrationPose;
            this->current_status = this->STATUS_NOT_SURE;
            this->ever_init = true;
            return this->current_status;
        }
        //已初始化:
        //bool speed_and_angular_rate_correct = checkSpeedAndAngularRate();
        //if(!speed_and_angular_rate_correct)
        //{
        //    this->current_status = this->STATUS_ERROR;
        //}
        //else
        //{
        //    this->current_status = this->STATUS_CORRECT;
        //}
        return this->current_status;

    }
    void initSurveillanceModule()
    {
        setModuleName("Localization Module");
    }
    int checkStateLegal()
    {
        return this->current_status;
    }
private:
    bool ever_init = false;
    geometry_msgs::PoseStamped lastPose;
    bool checkSpeedAndAngularRate(const geometry_msgs::PoseStamped& pose1,const geometry_msgs::PoseStamped& pose2);

};



#endif
