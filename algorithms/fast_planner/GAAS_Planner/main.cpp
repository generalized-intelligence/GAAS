#include <iostream>
#include "astar.h"
#include <map>

int main(int argc, char **argv) {
  Eigen::Vector3d a(1.22,2.32,2.42);
  Eigen::Vector3i b(1,2,2);
  
  Eigen::Vector3d c = a / 0.1;
  std::cout<<c<<std::endl;
  
  int i = 1;
  int k = 2;
  std::cout << i/2.0 << std::endl;
  //if (a==b)
    //std::cout << a / 0.1 << std::endl;
  std::cout << "Hello, world!" << std::endl;
  
  return 0;
}
