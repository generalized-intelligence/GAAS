#ifndef GOG_UTILS_H
#define GOG_UTILS_H
//part 1.time.
using namespace std;
unsigned long long micros()//instead of micros in Arduino.h
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    unsigned long long result = (unsigned long long)(tp.tv_sec * 1000000 + tp.tv_usec);
    //cout<<"micros():"<<result<<endl;
    //std::cout<<"result:"<<result<<std::endl;
    return result;
}
typedef unsigned long long time_us_t;

#endif
