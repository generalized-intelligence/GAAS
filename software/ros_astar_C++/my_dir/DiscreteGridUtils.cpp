#include<iostream>
#include<bits/stdc++.h> 
#include "DiscreteGridUtils.h"


using namespace std; 


DiscreteGridUtils::DiscreteGridUtils(double gridsiz)
{
    grid_size = gridsiz;
}


Pair DiscreteGridUtils::continuous_to_discrete(Pair pos)
{
    double a1 =  (int)((pos.first + (grid_size*0.5))/grid_size)-1;
    double b1 = (int)((pos.second.first + (grid_size*0.5))/grid_size)-1;
    double c1 = (int)((pos.second.second + (grid_size*0.5))/grid_size)-1;

    return make_pair(a1,make_pair(b1,c1));  
}

Pair DiscreteGridUtils::discrete_to_continuous_target(Pair grid_pos)
{
    double a2 = (grid_pos.first+0.5)*grid_size;
    double b2 = (grid_pos.second.first+0.5)*grid_size;
    double c2 = (grid_pos.second.second+0.5)*grid_size;

    return make_pair(a2,make_pair(b2,c2));  
}

/*
int main()
{
    DiscreteGridUtils dg(0.5);

    cout<<"hello"<<endl;

    //test
    Pair res1 = dg.continuous_to_discrete(make_pair(0.4,make_pair(0.4,0.4)));
    cout<<"res1" << res1.first<<" "<< res1.second.first<<" "<<res1.second.second<<endl;

    Pair res2 = dg.continuous_to_discrete(make_pair(-0.4,make_pair(-0.4,-0.4)));
    cout<<"res2" << res2.first<<" "<< res2.second.first<<" "<<res2.second.second<<endl;

    Pair res3 = dg.discrete_to_continuous_target(make_pair(1,make_pair(1,1)));
    cout<<"res3" << res3.first<<" "<< res3.second.first<<" "<<res3.second.second<<endl;

    Pair res4 = dg.discrete_to_continuous_target(make_pair(0,make_pair(0,0)));
    cout<<"res4" << res4.first<<" "<< res4.second.first<<" "<<res4.second.second<<endl;

    return 0;
}
*/

