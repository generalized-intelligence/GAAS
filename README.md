# GAAS: Towards L5 Autonomous Flying Cars, a Robust Framework Extends GAAS with Lidars!

![Star](https://img.shields.io/github/stars/generalized-intelligence/gaas?style=flat-square)![fork](https://img.shields.io/github/forks/generalized-intelligence/gaas?style=flat-square)![watch](https://img.shields.io/github/watchers/generalized-intelligence/gaas?style=flat-square)![BSD-3](https://img.shields.io/github/license/generalized-intelligence/gaas?style=flat-square)![twitter](https://img.shields.io/twitter/follow/GAAS_dev?style=social)


## About GAAS

**GAAS is an open-source program designed for fully autonomous VTOL(a.k.a flying cars) and drones.** GAAS stands for Generalized Autonomy Aviation System. We hope to accelerate human use of the sky through the full autonomy of flying vehicles. This project started in 2016 as a hobby for two students. 2019 we open-source the project and hope to develop GAAS full time in the long term.

**GAAS provides a fully autonomous flight platform based on lidar, HD-map relocalization, path planning, and other modules for aircraft.** In contrast to the autopilot technology previously available only for consumer-grade drones, GAAS aims for robust  fully autonomous flight for human-carrying, and can be easily combined with national air traffic control. At GAAS you can see many of the automotive-grade(AG) technologies that were previously only available in self-driving cars. The whole framework is coupled loosely so you can customize your own modules and easily add them to GAAS.

## Previews:

![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/algorithms/preview_imgs/gaas_algorithms_rviz_preview_20200401.png)

![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/algorithms/preview_imgs/gaas_algorithms_astar_planning_preview_20210409.png)

![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/algorithms/preview_imgs/gaas_algorithms_rqt_graph_20200401.png)

![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/algorithms/preview_imgs/gaas_algorithms_dynamic_objects_and_replanning.png)

A video has been uploaded to show the whole pipeline. You may need to download this [video](https://github.com/cyanine-gi/GAAS_contrib_resources/blob/main/demos/gaas_contrib_test1_20210419_compressed.mp4?raw=true).


## Differences between GAAS deprecated and new GAAS:

We use lidars, as main sensor, rather than vision algorithms.


GAAS deprecated is based on computer vision(CV), but fully vision-algorithms-based framework is not robust enough for autonomous flying.

For flying cars and large cargo drones, vision-based algorithms suffer from:

1. Lack of robustness, especially at night or over-exposed conditions. When air vehicles are flying at high speed, the localization is not stable enough which may cause severe accidents(vital to large air vehicles).

2. Computer vision is computationally expensive and does not easily run in real-time on mobile devices.

3. The neural network-based approach is accident-prone in extreme scenarios, and the failures are not easily reproducible.

These problems are not allowed to occur in manned flight scenarios.

Therefore, the introduction of LIDAR seems to be necessary at present. That's why we make new GAAS from scratch, and refactored all modules in cpp.

## Build with:

**Tested on OS: Ubuntu 18.04; PX4(for simulation only) 1.8.0.**

### step<1> Check your network status

    wget www.google.com

### step<2> tools

(optional) install **cuda 10.2** for all **gpu-based** algorithms, like icp_lidar_localization and the gpu version of ndt_localization.

You may need to **upgrade cmake to at least 3.13** for building package icp_lidar_localization.

    sudo apt install vim bwm-ng htop tmux git net-tools cmake-gui iotop curl

### step<3> docker(for simulation only)

    curl -fsSL https://get.docker.com | bash -s docker --mirror Aliyun

    sudo usermod -aG docker [your username]

    docker pull gaas/mavros-gazebo7-px4

### step<4> ros_melodic

    ./install_ros_melodic.sh

### step<5> opencv 3.4.5

    sudo apt install cmake-qt-gui
    
[Download opencv 3.4.5 and unzip]

    cd opencv-3.4.5/
    mkdir build&&cd build&&cmake-gui ..
    
[Configure your opencv cmake options in cmake-gui]
    
    make -j4&&sudo make install

### step<6> glog

    git clone https://github.com/google/glog.git
    cd glog
    git checkout -b v0.4.0
    mkdir build&&cd build
    cmake ..
    make 
    sudo make install

### step<7> pcl 1.8.0 build from source

[Download pcl 1.8.0 and unzip]

    cd pcl-1.8.0
    mkdir build&&cd build&&cmake-gui ..

[Configure your pcl cmake options in cmake-gui]

    make -j4
    sudo make install

### step<8> (optional) upgrade your gazebo for simulation

    cd GAAS/simulation
    ./upgrade_gazebo.sh



## Getting Started

To build the project, setup all dependencies, run:

    ./build_all.sh

To run GAAS_contrib algorithms:

    cd algorithms
    ./run_gaas_contrib_algorithms.sh

Start simulation (or play a rosbag instead):

    cd simulation&&./scripts/prepare_simulation.sh
    
or:

    rosbag play --clock [path_to_your_rosbag]

**And checkout your L5 flying car demo in simulation environment!**

## License

GAAS is under BSD 3-Clause License.

## Features

1. Simulation env with 32 lines lidar and stereo cameras.

2. Spinning lidar mapping and NDT matching localization.

Check out simulation/README.md to get more details of simulation env setup.

## Roadmap:

#### 1.  Gazebo simulation env construction, including spinning lidars and non-repetitive lidars and stereo cameras.

(1). Livox Horizon + Forward Stereo Camera --Done.

(2). Velodyne HDL-32 + Forward Stereo Camera --Done.

#### 2. Accelerate compiling and deployment of GAAS.

#### 3. Implement some LIDAR (mechanical/solid-state) based algorithms, and implement one key start in the simulation environment.

## Checklist:

(1). Lidar Points to Image Projection-- Done.

(2). Euclidean Cluster Extraction. --Done.

(3). Global Coordinate based HD-Map Building. --Done.

(4). NDT Lidar Localization(CPU/Cuda) --Done.

(5). Downsampling Node --Done.

(6). A* Path Planner --Done.

(7). Refactored px4 Offboard Commander --Done.

(8). Dynamic Obstacles Generation and Replanning --Done.

(9). Jetson AGX Xavier Adaptation --Done.

(10). Interactive GUI Target Selector in HD-maps --Done.

(11). Multiple Submaps Switching --TODO

(12). IMU-Preintegration and High-Frequency Localization --Done.

(13). VTOL Mode Switching --TODO.

(14). Decentralized Robust Ground Control Station --TODO.

(15). Generalized Flight Controller State Management --Done.

(16). PX4 State Reporter --Done.

(17). HUD Module --Done.

(18). Cuda-based Multiple Lidar Pointclouds ICP Localization --Done.

(19). Ground Points Removal Preprocessing --Done.

(20). System State Surveillance Service --Done.

(21). HTTP Server on Ground Control Station --TODO.

(22). Multiple Spinning Lidar Support --Done.

(23). Airsim Simulation Env Support --Done.

## Current status:

Adding logics for flight stage manager module. Including flight stage transfer service clients(triggered by mission config file) and servers(including localization module, flight control commander module and target navigation module.) 
