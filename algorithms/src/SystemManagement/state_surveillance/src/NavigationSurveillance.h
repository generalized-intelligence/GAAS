#ifndef NAVIGATION_SURVEILLANCE
#define NAVIGATION_SURVEILLANCE
#include "SurveillanceModuleAbstract.h"



class NavigationSurveillanceModule:SurveillanceModuleAbstract
{

    void initSurveillanceModule()
    {
        setModuleName("Navigation Module");
    }
    int checkStateLegal()
    {
        return this->current_status;
    }
};



#endif
