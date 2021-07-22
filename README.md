# GAAS: Towards L5 Autonomous Flying Cars, a Robust Framework Extends GAAS with Lidars!

![Star](https://img.shields.io/github/stars/generalized-intelligence/gaas?style=flat-square)![fork](https://img.shields.io/github/forks/generalized-intelligence/gaas?style=flat-square)![watch](https://img.shields.io/github/watchers/generalized-intelligence/gaas?style=flat-square)![BSD-3](https://img.shields.io/github/license/generalized-intelligence/gaas?style=flat-square)![twitter](https://img.shields.io/twitter/follow/GAAS_dev?style=social)


## About GAAS

**GAAS is an open-source program designed for fully autonomous VTOL(a.k.a flying cars) and drones.** GAAS stands for Generalized Autonomy Aviation System. We hope to accelerate human use of the sky through the full autonomy of flying vehicles. This project started in 2016 as a hobby for two students. 2019 we open-source the project and hope to develop GAAS full time in the long term.

**GAAS provides a fully autonomous flight platform based on lidar, HD-map relocalization, path planning, and other modules for aircraft.** In contrast to the autopilot technology previously available only for consumer-grade drones, GAAS aims for robust  fully autonomous flight for human-carrying, and can be easily combined with national air traffic control. At GAAS you can see many of the automotive-grade(AG) technologies that were previously only available in self-driving cars. And there is a wealth of extensibility to help you customize your development to your goals.

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

### step<1> Check your network status

    ping github.com

### step<2> tools

    sudo apt install vim bwm-ng htop tmux git net-tools cmake-gui

### step<3> docker(for simulation only)

    curl -fsSL https://get.docker.com | bash -s docker --mirror Aliyun

    sudo usermod -aG docker gi

    docker pull gaas/mavros-gazebo7-px4

### step<4> ros_melodic

    ./install_ros_melodic.sh

### step<5> opencv 3.4.5

install cmake-gui make sudo make install

### step<6> glog

https://github.com/google/glog.git  git checkout -b v0.4.0 

cmake  make  make install

### step<7> pcl 1.8 build from source

### step<8>(for icp cuda only) update your cmake to 3.14.7 and install cuda 10.2

OS: Ubuntu 18.04

PX4(simulation only) 1.8.0

## Getting Started

To build the project, setup all dependencies and run:

    ./build_all.sh

To run GAAS_contrib algorithms:

    cd algorithms
    ./run_gaas_contrib_algorithms.sh

**And checkout your L5 flying car demo in simulation environment!**

## License

GAAS is under BSD 3-Clause License.

## Features and Roadmap

Please Checkout by this link


