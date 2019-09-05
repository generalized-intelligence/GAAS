# User Guide

## ygz slam with ROS wrapper

    This project is heavily based on https://github.com/gaoxiang12/ygz-stereo-inertial
    
    Special thanks to gaoxiang@https://github.com/gaoxiang12

To use ygz slam with ROS wrapper, please use the following method.

## Dependencies
    
    NOTE: 
    
    This package requires an Opencv version higher than 3.4.5 and opencv contirb is required as well.
    Before building opencv, make sure you have set ENABLE_CXX11 and WITH_QT to true. 
    Also, before build g2o, remember to install suitesparse, if you are running ubuntu or debian based system, 
    you can use
    
    libsuitesparse-dev
   
    please install the following dependencies:
    
    1. Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin
    2. Eigen3: sudo apt-get install libeigen3-dev
    3. g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake
    4. OpenCV: sudo apt-get install libopencv-dev
    5. glog (for logging): sudo apt-get install libgoogle-glog-dev
    6. ros-kinetic: refer to the github front page

## Compile

    In the root folder:
    ./generate.sh
    
## Usage

    ./bin/EurocStereoVIO_ros ./examples/SimulationCamera.yaml

    Or

    sh run.sh

# 中文使用指南

## ygz slam with ROS wrapper

    本项目大部分基于 https://github.com/gaoxiang12/ygz-stereo-inertial
    
    非常感谢 高博@https://github.com/gaoxiang12 对此项目的支持

若想使用此项目， 请参考下列步骤。


## 依赖

    请安装下列依赖：
    
    1. Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin
    2. Eigen3: sudo apt-get install libeigen3-dev
    3. g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake
    4. OpenCV: sudo apt-get install libopencv-dev
    5. glog (for logging): sudo apt-get install libgoogle-glog-dev
    6. ros-kinetic: refer to the github front page
    
    
## 编译

    在项目根目录:
    ./generate.sh
    
## 使用

    ./bin/EurocStereoVIO_ros ./examples/SimulationCamera.yaml 

    或

    sh run.sh
