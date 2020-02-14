#ifndef __DISCRETEGRID_H_
#define __DISCRETEGRID_H_

#include <iostream>
#include <eigen3/Eigen/Eigen>

using namespace std;  


class DiscreteGrid
{
    public:

    double grid_size;

    DiscreteGridUtils(double gridsiz = 0.3);

    Eigen::Vector3d continuous_to_discrete(Eigen::Vector3d pos);
    Eigen::Vector3d discrete_to_continuous_target(Eigen::Vector3d grid_pos);
};

#endif // __DISCRETEGRID_H_

