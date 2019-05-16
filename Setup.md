## Setup

NOTE: Tested in ubuntu 16.04 and ros-kinetic. For modules that require further development, we recommend to install from source and replace corresponding ROS packages.

### 1. Install ROS-Kinetic

We recommend installing ros-kinetic-desktop-full from the following address:
    http://wiki.ros.org/kinetic/Installation/Ubuntu

### 2. Install Octomap-Server and mavros
    sudo apt install python-catkin-tools ros-kinetic-mavlink ros-kinetic-geographic-msgs ros-kinetic-octomap-ros libgeographic-dev geographiclib-tools ros-kinetic-control-toolbox libpopt-dev
    cd ~/catkin_ws/src
    git clone https://github.com/OctoMap/octomap_mapping.git
    git clone https://github.com/mavlink/mavros.git
    cd ..
    catkin build
    source ~/catkin_ws/devel/setup.bash

### 3. Gazebo

If you followed step 1 and has installed ros-kinetic-desktop-full, you should already have gazebo7 installed. Now install the corresponding ros plugin:
    
    sudo apt install ros-kinetic-gazebo-*
    
If you would like to use another version of Gazebo, please see http://gazebosim.org/ for different versions of the plugin:
    
    sudo apt install ros-kinetic-<your gazebo version>-*

### 4. PCL

Install at the following address:
    
    http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php

### 5. YGZ-slam

Detailed information can be found in the folder: https://github.com/gaoxiang12/ygz-stereo-inertial

### 6. PX4

NOTE: Tested on px4 v1.8.0
    
    mkdir ~/px4 && cd ~/px4
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    git checkout v1.8.0
    
    
-----
## 安装

NOTE: 目前只在 Ubuntu 16.04 以及 ros-kinetic 环境下测试通过。如需对 GAAS 进行二次开发，以下安装步骤中所有想要更改的组件源码安装，并在 ROS 的对应接口进行替换即可。

### 1. 安装 ROS-Kinetic

建议安装 ros-kinetic-desktop-full; 可按照此地址进行安装：
	http://wiki.ros.org/kinetic/Installation/Ubuntu.

### 2. 安装 Octomap-Server 以及 mavros

	cd ~/catkin_ws/src
	git clone https://github.com/OctoMap/octomap_mapping.git
	git clone https://github.com/mavlink/mavros.git
	cd ..
	catkin build
	source ~/catkin_ws/devel/setup.bash

### 3. Gazebo

如果您在第一步安装了 ros-kinetic-desktop-full, 您已经具有了 gazebo7；, 接下来请安装对应 ros plugin:

	sudo apt install ros-kinetic-gazebo-*

如果您想使用其他版本的 gazebo, 请参考：http://gazebosim.org/, 并安装对应版本的 ros plugin:

	sudo apt install ros-kinetic-<your gazebo version>-*


### 4. PCL

请参考此链接安装：
	http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php


### 5. YGZ-slam

请参考此链接安装:
	https://github.com/gaoxiang12/ygz-stereo-inertial


### 6. PX4
NOTE:  仅在 px4 v1.8.0 上进行过测试。
    
    mkdir ~/px4 && cd ~/px4
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    git checkout v1.8.0

