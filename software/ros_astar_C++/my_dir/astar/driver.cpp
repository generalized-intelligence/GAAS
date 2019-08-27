#include<iostream>
#include<bits/stdc++.h> 
#include <set> 
#include<ros/ros.h>
#include"driver.h"

using namespace std; 


Driver::Driver()
{
    TIME_DELAY_THRESHOLD = 500;
    obs_set_last_update_time = ros::Time::now().toSec();
} 

void Driver::set_obstacle_set(set <Pair> obstacleset)
{
    obstacle_set = obstacleset;
    obs_set_last_update_time = ros::Time::now().toSec();
}

setpair  Driver::get_obstacles_around()
{
    double current_time = ros::Time::now().toSec();
    if(current_time - obs_set_last_update_time > TIME_DELAY_THRESHOLD)
    {
        cout<<"Warning:Buffer timeout!Delay:"<<current_time-obs_set_last_update_time<<endl;
    }
    return obstacle_set;
}


/*
int main()
{
    return 0;
}
*/
