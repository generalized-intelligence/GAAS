# Generalized Autonomy Aviation System


![lisence](https://img.shields.io/github/license/generalized-intelligence/GAAS.svg?style=for-the-badge)
![issues](https://img.shields.io/github/issues-raw/generalized-intelligence/GAAS.svg?style=for-the-badge)



![star](https://img.shields.io/github/stars/generalized-intelligence/GAAS.svg?style=social)

![twitter](https://img.shields.io/twitter/follow/GAAS_ooo.svg?style=social)

![future](https://img.shields.io/badge/Let's%20back%20to-the%20Future!-blue.svg)


You Can Chat and Find Support at: [![Join the chat at https://gitter.im/GAAStalk/community](https://badges.gitter.im/GAAStalk/community.svg)](https://gitter.im/GAAStalk/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Or watch the step-by-step [tutorial](https://github.com/generalized-intelligence/GAAS/blob/master/demo/tutorial_1/README.MD)

Or follow the lastest news about the project at our [Medium Publication](https://medium.com/generalized-intelligence)
# GAAS

<img src="https://s2.ax1x.com/2019/01/31/k1TWUU.jpg" align="right" width="300px" alt="GA">

> Generalized Autonomy Aviation System (GAAS, pronounciate as "gas") is an open source project dedicated to autonomy flight and operating control system for futuristic aerial vehicles.

> Imagine commuting with "flying car", having food and packages delivered by drones to your door. To contribute to the described future, we would like to get the two most fundamental issues out of the way: the safety of aerial vehicles, and the demand for skilled pilots. 

> Our long-term vision is to implement GAAS in passenger carrying aerial vehicles (could be "flying car"). The first step of this vision is to make Unmanned Aerial Vehicles truly "unmanned", so drones can be better utilized. We also just want to provide an easy-to-use infrastructure for programmers and engineers to program and build drone products.

> The current version of GAAS includes Simultaneous Localization and Mapping (SLAM), obstacle avoidance, navigation and flight control, supporting manned and unmanned multi-rotor drones and helicopters. Next step, we will also be supporting VTOL and eVTOL.

> 泛化自动驾驶飞行器系统 Generalized Autonomy Aviation System（以下简称 GAAS）是一个面向未来飞行器设计的开源驾驶以及运行管理系统。

> 想象一个每天用飞行汽车出门，无人机送餐和快递的未来。我们希望为这样的未来解决一些基础性的问题：满足对安全性的高要求并消除飞行员以及飞手的需求。

> 我们第一步的目标是让无人机做到真正无人，从而能够为人类进行更多的工作。并且可以让更多程序员和工程师更简单的为无人机编写程序和制作产品。长期目标是将 GAAS 应用在载人飞行器上（也许是飞行汽车）。

>当前版本的 GAAS 包含 SLAM、避障路径规划、飞行控制模块；用于支持无人和有人的旋翼和直升机驾驶。下一步计划支持 VTOL 和 eVTOL。

![UAV demo](https://s2.ax1x.com/2019/02/25/kIZ3vj.jpg)

An example of the hardware


## Setup

NOTE: Tested in ubuntu 16.04 and ros-kinetic

### 1. Install ROS-Kinetic

We recommend installing ros-kinetic-desktop-full from the following address:
    http://wiki.ros.org/kinetic/Installation/Ubuntu

### 2. Install Octomap-Server and mavros

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

## Sample Use Case

Obstacle avoidance, 3D tap-to-fly and navigation (independent of GPS):

    step<1> If used indoor or require accurate localization, add SLAM in ROS launch file, otherwise, skip this step.

    step<2> For outdoor usage, configure location points in GPS. For indoor usage, configure location points in local East North Up (ENU)(relative to the 3D model).

    step<3> Add collision avoidance module to ROS launch file.

    step<4> Turn on the vehicle, wait for PX4 to finish the pre-flight check.

    step<5> Launch Mavros.

    step<6> Send the vehicle out for the mission.

Upcoming features:

	1. Tracking and Following with Obstacle Avoidance

	2. Computer Vision enabled Accurate Landing

	3. Auto-Search and 3D Modelling

	4. Multi-Drone Collaboration

## Development Setup

For modules that require further development, we recommend to:

Install from source

Replace corresponding ROS packages

## Meta

Project initialized by Generalized Intelligence

Distributed under the BSD 3-Clause license. See ``LICENSE`` for more information.

[GAAS GitHub](https://github.com/generalized-intelligence/GAAS)

## Special Thanks

It is worth mentioning that we did not build everything from scratch, but on top of the solid foundations built by pioneers in the field. We would like to thank communities such as [PX4](https://px4.io) and [Dronecode](https://www.dronecode.org) for constantly pushing the industry foward. What they have built are what allowed us to build GAAS!

## Contribute

Please follow CONTRIBUTING.md

Il Vole

---

## Setup

NOTE: 目前只在 Ubuntu 16.04 以及 ros-kinetic 环境下测试通过。

### 1.安装 ROS-Kinetic

建议安装 ros-kinetic-desktop-full; 可按照此地址进行安装：
	http://wiki.ros.org/kinetic/Installation/Ubuntu.

### 2.安装 Octomap-Server 以及 mavros

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

## 使用案例

1.带避障，不依赖 GPS 的路径规划指点飞行。

	step<1> 如果是室内环境使用或需要高精度飞行，ROS launch file 加入 SLAM；否则跳过此步骤。

	step<2> 对于室外环境，配置 GPS 轨迹(使用地图)。对于室内部分，在 3D 模型中配置轨迹。

	step<3> ROS launch file 加入避障部分。

	step<4> 启动飞行器，等待 PX4 自检通过。

	step<5> 启动 Mavros。

	step<6> 开始执行任务。

即将推出：

	1. 带避障的视觉跟踪任务

	2. 视觉精准降落

	3. 区域自动探索建模

	4. 多机协同执行任务


## 开发配置

Setup 部分中所有想要更改的组件源码安装。

在 ROS 的对应接口进行替换即可。


## Meta

泛化智能 Generalized Intelligence 出品。

本项目受 BSD 3-Clause 协议保护。点击``LICENSE`` 了解更多
[GAAS GitHub](https://github.com/generalized-intelligence/GAAS)

## Special Thanks

我们的项目并不是从零开始，而是站在了过去十余年无人机行业先驱的肩膀上。非常感谢 PX4 与 Dronecode 等组织对无人机行业发展的推动，让我们今天得以有机会制作 GAAS!

## Contribute

请参阅 CONTRIBUTING.md

Il Vole
