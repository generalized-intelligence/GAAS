#ifndef DRIVER_H
#define DRIVER_H

#include<iostream>
#include<bits/stdc++.h> 
#include <set> 
#include<ros/ros.h>
#include "astar.h"


using namespace std; 

typedef pair<double,  pair<double, double> > Pair; 
typedef set <Pair> setpair;

class Driver
{
    public:
    double TIME_DELAY_THRESHOLD;
    set <Pair> obstacle_set;
    double obs_set_last_update_time;

    Driver();
    void set_obstacle_set(set <Pair> obstacleset);
    setpair  get_obstacles_around();
};

#endif