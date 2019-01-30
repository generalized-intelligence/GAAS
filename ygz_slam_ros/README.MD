## ygz slam with ROS wrapper

    This project is heavily based on https://github.com/gaoxiang12/ygz-stereo-inertial

## To use ygz slam with ROS wrapper

## Dependencies

    please install the following dependencies:
    1. Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin
    2. Eigen3: sudo apt-get install libeigen3-dev
    3. g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake
    4. OpenCV: sudo apt-get install libopencv-dev
    5. glog (for logging): sudo apt-get install libgoogle-glog-dev
    6. ros-kinetic: refer to the github front page

## Compile

    In the root folder: ./generate.sh
    
## Usage

    ./bin/EurocStereoVIO_ros ./examples/SimulationCamera.yaml
