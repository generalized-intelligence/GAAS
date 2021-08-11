#ifndef NAVIGATION_SURVEILLANCE
#define NAVIGATION_SURVEILLANCE
#include "SurveillanceModuleAbstract.h"



class NavigationSurveillanceModule:public SurveillanceModuleAbstract
{

    virtual void initSurveillanceModule()
    {
        setModuleName("Navigation Module");
    }
    virtual int checkStateLegal()
    {
        return this->current_status;
    }
};



#endif
