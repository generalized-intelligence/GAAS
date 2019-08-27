#ifndef DISCRETEGRIDUTILS_H
#define DISCRETEGRIDUTILS_H

#include<iostream>
#include<bits/stdc++.h> 


using namespace std; 

typedef pair<double,  pair<double, double> > Pair; 


class DiscreteGridUtils
{
    public:

    double grid_size;

    DiscreteGridUtils(double gridsiz = 0.3);

    Pair continuous_to_discrete(Pair pos);
    Pair discrete_to_continuous_target(Pair grid_pos);
};

#endif