#ifndef HEADER_TIMER_H_FILE
#define HEADER_TIMER_H_FILE


#include <chrono>
#include <glog/logging.h>
//using namespace chrono;

struct ScopeTimer
{
    std::chrono::high_resolution_clock::time_point begin,end;

    std::string name;
    bool use;
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc; 
    ScopeTimer(std::string name_,bool use_=true,SCALE _sc=MSEC)
    {
        name=name_;
        use=use_;
        sc=_sc;
        begin= std::chrono::high_resolution_clock::now();
    }
    void watch(const std::string& info_desc)
    {
        if(use){
            end= std::chrono::high_resolution_clock::now();
            double fact=1;
            std::string str;
            switch(sc)
            {   
            case NSEC:fact=1;str="ns";break;
            case MSEC:fact=1e6;str="ms";break;
            case SEC:fact=1e9;str="s";break;
            };  
            LOG(INFO) << "Time cost till("<<info_desc<<")= "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count())/fact<<str<<std::endl; ;
        }
    }
    ~ScopeTimer()
    {
        if (use){
            end= std::chrono::high_resolution_clock::now();
            double fact=1;
            std::string str;
            switch(sc)
            {
            case NSEC:fact=1;str="ns";break;
            case MSEC:fact=1e6;str="ms";break;
            case SEC:fact=1e9;str="s";break;
            };

            LOG(INFO) << "Time cost("<<name<<")= "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count())/fact<<str<<std::endl; ;
        }   
    }   
};
#endif
