#ifndef __BRESENHAM3D_H_
#define __BRESENHAM3D_H_

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

#endif // __BRESENHAM3D_H_
