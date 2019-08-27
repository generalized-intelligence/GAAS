#ifndef PATHPRUNING_H
#define PATHPRUNING_H


#include <iostream>
#include <set> 
#include <vector>
#include <math.h>
#include<bits/stdc++.h>
#include <algorithm>

#include "bresenham3d.h"
using namespace std;

class PathPruning
{
    public:
    double obstacle_distance;
    vector<Pair> new_path;
    vector<Pair> final_path;


    PathPruning(double obstacle_dist = 10);

    vector<Pair> remove_collinear_points(vector<Pair> original_path);

    vector<Pair> Bresenham3D(Pair p1, Pair p2);


    vector<Pair> path_pruning_bresenham3d(vector<Pair> path, set<Pair> local_obstacle);
/*
    vector<Pair> BSplinePathSmoothing(vector<Pair> path)
    {
        return;
    }*/

    vector<Pair> Remove(vector<Pair> v);

    double distance(Pair pt1, Pair pt2);

    void print_pair(Pair p1);
};

#endif