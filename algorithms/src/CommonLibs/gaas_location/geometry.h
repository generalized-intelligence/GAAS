#ifndef GAAS_COMMON_LIBS_GEOMETRY_H
#define GAAS_COMMON_LIBS_GEOMETRY_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>


inline double getEuclideanDistance(double x1,double y1,double z1,double x2,double y2,double z2)
{
    return sqrt( pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
}

inline double getEuclideanDistance(Eigen::Vector3f v1,Eigen::Vector3f v2)
{
    return (v1-v2).norm();
}
inline double getEuclideanDistance(Eigen::Vector3d v1,Eigen::Vector3d v2)
{
    return (v1-v2).norm();
}
inline double getEuclideanDistance2D(Eigen::Vector3d v1,Eigen::Vector3d v2)
{
    v1[2]=0;
    v2[2]=0;
    return getEuclideanDistance(v1,v2);
}




#endif
