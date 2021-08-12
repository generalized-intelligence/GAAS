#ifndef FLIGHT_CONTROLLER_SURVEILLANCE_H
#define FLIGHT_CONTROLLER_SURVEILLANCE_H

#include "SurveillanceModuleAbstract.h"
#include "gaas_msgs/GAASSystemManagementFlightControllerState.h"
#include <geometry_msgs/PoseStamped.h>
#include "utils.h"

class FlightControllerModule:public SurveillanceModuleAbstract
{
    int runPipelineAndGetState(const gaas_msgs::GAASSystemManagementFlightControllerStateConstPtr& pFCState)
    //int runPipelineAndGetState(const geometry_msgs::PoseStampedConstPtr& pFCPose)
    {
        if(!this->ever_init)
        {
            this->lastFCState = *pFCState;
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
    virtual void initSurveillanceModule()
    {
        setModuleName("Flight Controller Module");
    }
    virtual int checkStateLegal()
    {
        return this->current_status;
    }
private:
    bool ever_init = false;
    gaas_msgs::GAASSystemManagementFlightControllerState lastFCState;
    geometry_msgs::PoseStamped lastPose;//手动计算....
    bool checkSpeedAndAngularRate(const geometry_msgs::PoseStamped& pose1,const geometry_msgs::PoseStamped& pose2);

};



#endif
