/*
 * Copyright (c) 2020 <copyright holder> <email>
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

#ifndef __PATH_PRUNING_H_
#define __PATH_PRUNING_H_

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
#endif // __PATH_PRUNING_H_
