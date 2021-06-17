echo "Building ROS nodes"

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
