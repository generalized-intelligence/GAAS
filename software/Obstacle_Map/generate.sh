mkdir build
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PWD}
echo ${PWD}
cd build
cmake ..
make -j4
cd ..
