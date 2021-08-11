#ifndef SURVEILLANCE_MODULE_ABSTRACT_H
#define SURVEILLANCE_MODULE_ABSTRACT_H

#include <memory>
#include <string>
#include <glog/logging.h>

class SurveillanceModuleAbstract
{
public:
    const static int STATUS_CORRECT = 0;//确定可靠.
    const static int STATUS_NOT_SURE = 1;//暂时不能确定是否可靠.
    const static int STATUS_WARNING = 2;
    const static int STATUS_ERROR = 3;


    std::string module_name="abstract module";
    int current_status=STATUS_NOT_SURE;
    void setModuleName(const std::string& name)
    {
        this->module_name = name;
    }
    virtual void initSurveillanceModule()=0;
    virtual int checkStateLegal()=0;
    virtual ~SurveillanceModuleAbstract()
    {
        ;
    }
    virtual void SURVEILLANCE_LOG_STATUS(const std::string& state_description)
    {
        if(current_status == STATUS_NOT_SURE)
        {
            LOG(INFO)<<module_name<<" state: STATUS_NOT_SURE"<<" "<<state_description;
        }
        else if(current_status == STATUS_WARNING)
        {
            LOG(WARNING)<<module_name<<" state: STATUS_WARNING"<<" "<<state_description;
        }
        else if(current_status == STATUS_ERROR)
        {
            LOG(ERROR)<<module_name<<" state: STATUS_ERROR"<<" "<<state_description;
        }
    }
};


#endif
