#include "Pipeline.h"
int main(int argc,char**argv)
{
    auto pImg1 = make_shared<cvMatT>();
    auto pImg2 = make_shared<cvMatT>();
    *pImg1 = IMREADcvMatT(argv[1]);
    *pImg2 = IMREADcvMatT(argv[2]);

    run_all_pipeline(pImg1,pImg2);
    //ScopeTimer ("mesh generated 100 times.begin.");
    ScopeTimer t_100_times("mesh generates for 100 times");
    for(int i = 0;i<100;i++)
    {
        run_all_pipeline(pImg1,pImg2);
    }
    t_100_times.watch("end");

    return 0;
}
