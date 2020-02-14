#include "path_optimization/bresenham3d.h"
using namespace std;

vector<Pair> Bresenham3D(Pair p1, Pair p2)
{
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