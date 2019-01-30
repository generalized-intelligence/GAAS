echo "Building ROS nodes"

mkdir build || true
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j