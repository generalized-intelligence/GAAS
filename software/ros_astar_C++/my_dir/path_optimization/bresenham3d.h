#ifndef BRESENHAM3D_H
#define BRESENHAM3D_H

#include <iostream>
#include <set> 
#include <vector>
#include <math.h>
#include<bits/stdc++.h> 

using namespace std;

typedef pair<double,  pair<double, double> > Pair; 
typedef pair<string, Pair> sPair;
typedef set <Pair> setpair;

vector<Pair> Bresenham3D(Pair p1, Pair p2);

#endif