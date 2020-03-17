#ifndef __SCOPETIMER_H__
#define __SCOPETIMER_H__

#include <chrono>
#include <glog/logging.h>

struct ScopeTimer
{
    std::chrono::high_resolution_clock::time_point init_time, start, end;
    
    double cost_time;

    std::string name;
    bool use;
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc; 
    
    double fact;
    std::string str;
    
    ScopeTimer(std::string name_,bool use_=true,SCALE _sc=MSEC)
    {
        name=name_;
        use=use_;
        sc=_sc;
        init_time= std::chrono::high_resolution_clock::now();
	start = std::chrono::high_resolution_clock::now();
	cost_time = 0;
	
	switch(sc)
        {   
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };
    }
    void watch(const std::string& info_desc, bool watch_cost_ =true)
    {
        if(use)
	{
          end= std::chrono::high_resolution_clock::now();
	    
	  if(watch_cost_ == false)
	  {
            LOG(INFO) << "Time cost till("<<info_desc<<")= "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-init_time).count())/fact<<str<<std::endl;
	  }  
	  else
	  {
	    LOG(INFO) << "Time cost till("<<info_desc<<")= "<<cost_time<<str<<std::endl;
	  }
	 }
    }
    
    void setStartPoint()
    {
      start = std::chrono::high_resolution_clock::now();
    }
    
    void stopClockAndUpdateCost()
    {
      end= std::chrono::high_resolution_clock::now();
      cost_time += double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count())/fact;
      
    }
    
    ~ScopeTimer()
    {
        if (use){
            end= std::chrono::high_resolution_clock::now();
            LOG(INFO) << "Time cost("<<name<<")= "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-init_time).count())/fact<<str<<std::endl;
        }   
    }   
};

#endif // __SCOPETIMER_H__
