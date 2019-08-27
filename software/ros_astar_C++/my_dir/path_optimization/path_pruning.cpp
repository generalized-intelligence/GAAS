#include <iostream>
#include <set> 
#include <vector>
#include <math.h>
#include<bits/stdc++.h>
#include <algorithm>

#include "bresenham3d.h"
#include "path_pruning.h"

using namespace std;




PathPruning::PathPruning(double obstacle_dist)
{
    obstacle_distance = obstacle_dist;
    //cout<<"In the constructor"<<obstacle_distance<<endl;
}

vector<Pair> PathPruning::remove_collinear_points(vector<Pair> original_path)
{
    double distance13, distance12, distance23;
    original_path = Remove(original_path);
    
    int length = original_path.size();
    new_path.push_back(original_path[0]);

    for(int i=2; i<length; i++)
    {
        distance13 = distance(original_path[i], original_path[i-2]);
        distance12 = distance(original_path[i-1], original_path[i-2]);
        distance23 = distance(original_path[i], original_path[i-1]);

        if(abs(distance13 - distance12 - distance23) < 0.001)
        {
            continue;
        }
        else
        {
            new_path.push_back(original_path[i-1]);
        }
    }

    new_path.push_back(original_path[original_path.size()-1]);
    return new_path; 
}

vector<Pair> PathPruning::Bresenham3D(Pair p1, Pair p2)
{
//print_pair(p1);
//print_pair(p2);
double x1 = p1.first;
double y1 = p1.second.first;
double z1 = p1.second.second;

double x2 = p2.first;
double y2 = p2.second.first;
double z2 = p2.second.second;

vector<Pair> ListOfPoints;
ListOfPoints.push_back(p1);


double dx = abs(x2 - x1);
double dy = abs(y2 - y1);
double dz = abs(z2 - z1);
//cout<<dx<<" "<<dy<<" "<<dz<<endl;

int xs, ys, zs;

if(x2>x1)
    xs = 1;
else
    xs = -1;

if (y2 > y1)
    ys = 1;
else
    ys = -1;

if (z2 > z1)
    zs = 1;
else
    zs = -1;

double temp1;
double temp2;
// Driving axis is X-axis"
if(dx >= dy && dx >= dz)
{
    temp1 = 2*dy - dx;
    temp2 = 2*dz - dx;
    while(x1!=x2)
    {
        x1  = x1 + xs;
        if(temp1 >= 0)
        {
            y1 = y1 + ys;
            temp1 = temp1 - 2*dx;
        }
        if(temp2 >= 0)
        {
            z1 = z1 + zs;
            temp2 = temp2 - 2*dx;
        }
        temp1 = temp1 + 2*dy;
        temp2 = temp2 + 2*dz;
        ListOfPoints.push_back(make_pair(x1,make_pair(y1,z1)));
    }
}

//Driving axis is Y-axis"
else if (dy >= dx && dy >= dz)
{
    temp1 = 2*dx - dy;
    temp2 = 2*dz - dy;
    while(y1!=y2)
    {
        y1  = y1 + ys;
        if(temp1 >= 0)
        {
            x1 = x1 + xs;
            temp1 = temp1 - 2*dy;
        }
        if(temp2 >= 0)
        {
            z1 = z1 + zs;
            temp2 = temp2 - 2*dy;
        }
        temp1 = temp1 + 2*dx;
        temp2 = temp2 + 2*dz;
        ListOfPoints.push_back(make_pair(x1,make_pair(y1,z1)));
    } 
}

//Driving axis is Z-axis"
else
{
    temp1 = 2*dy - dz;
    temp2 = 2*dx - dz;
    while(z1!=z2)
    {
        z1  = z1 + zs;
        if(temp1 >= 0)
        {
            y1 = y1 + ys;
            temp1 = temp1 - 2*dz;
        }
        if(temp2 >= 0)
        {
            x1 = x1 + xs;
            temp2 = temp2 - 2*dz;
        }
        temp1 = temp1 + 2*dy;
        temp2 = temp2 + 2*dx;
        ListOfPoints.push_back(make_pair(x1,make_pair(y1,z1)));
    }
}

return ListOfPoints;
}


vector<Pair> PathPruning::path_pruning_bresenham3d(vector<Pair> path, set<Pair> local_obstacle)
{
    Pair start_position = path[0];
    final_path.push_back(start_position);
    Pair last_point = start_position;
    vector<Pair> bresenham_path;
    //cout<<"In the funcion"<<obstacle_distance<<endl;

    for(int i=0; i<path.size(); i++)
    {
        bresenham_path = Bresenham3D(start_position, path[i]);
        //int count = 0;
        
        for (set<Pair>::iterator it=local_obstacle.begin(); it!=local_obstacle.end(); ++it)
        {
            for(int j=0; j<bresenham_path.size(); j++)
            {
                //if((*it).first == 4 && (*it).second.first == 3 && (*it).second.second == 7 && bresenham_path[j].first == 4 && bresenham_path[j].second.first == 2 && bresenham_path[j].second.second == 0)
                //{
                    //  cout<<distance(*it, bresenham_path[j])<<" "<<obstacle_distance<<endl;
                //} 
                //cout<<distance(*it, bresenham_path[j])<<" "<<obstacle_distance<<endl;
                if(distance(*it, bresenham_path[j]) < obstacle_distance)
                {
                    //count++;
                    final_path.push_back(last_point);
                    final_path.push_back(path[i]);
                    start_position = path[i];
                    //cout<<count<<endl;
                }
            }
        }
        last_point = path[i];
    }
    final_path.push_back(path[path.size()-1]);
    return final_path;
}
/*
vector<Pair> BSplinePathSmoothing(vector<Pair> path)
{
    return;
}*/

vector<Pair> PathPruning::Remove(vector<Pair> v)
{
vector<Pair>::iterator itr = v.begin();
set<Pair> s;

for (auto curr = v.begin(); curr != v.end(); ++curr) 
{
    if (s.insert(*curr).second)
        *itr++ = *curr;
}

v.erase(itr, v.end());
return v;
}

double PathPruning::distance(Pair pt1, Pair pt2)
{
    return(sqrt(pow((pt1.first - pt2.first),2)+pow((pt1.second.first - pt2.second.first),2)+pow((pt1.second.second - pt2.second.second),2)));
}

void PathPruning::print_pair(Pair p1)
{
    cout<<p1.first<<" "<<p1.second.first<<" "<<p1.second.second<<endl;
}

/*
int main()
{
    vector<Pair> path;
    for(int i=0; i<10; i++)
    {
        path.push_back(make_pair(i,make_pair(0,0)));
    }
    for(int i=0; i<10; i++)
    {
        path.push_back(make_pair(9,make_pair(i,0)));
    }
    for(int i=0; i<5; i++)
    {
        path.push_back(make_pair(9,make_pair(9,i)));
    }

    set<Pair> obstacle;
    for(int i=3; i<5; i++)
    {
        for(int j=3; j<5; j++)
        {
            for(int k=7;k<16;k++)
            {
                obstacle.insert(make_pair(i,make_pair(j,k)));
            }
        }
    }

    PathPruning p(7.1);

    //for (set<Pair>::iterator it=obstacle.begin(); it!=obstacle.end(); ++it)
    //{
    //    cout<<(*it).first<<" "<<(*it).second.first<<" "<<(*it).second.second<<endl;
    //}
    
     vector<Pair> a = p.remove_collinear_points(path);
    cout<<"a path"<<endl;
    for(int i=0; i<a.size(); i++)
    {
        cout<<a[i].first<<" "<<a[i].second.first<<" "<<a[i].second.second<<endl;
    }

    vector<Pair> b = p.path_pruning_bresenham3d(path, obstacle);
    cout<<"b path"<<endl;
    for(int i=0; i<b.size(); i++)
    {
        cout<<b[i].first<<" "<<b[i].second.first<<" "<<b[i].second.second<<endl;
    }


    return 0;
}
*/