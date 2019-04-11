# Generalized Autonomy Aviation System


![lisence](https://img.shields.io/github/license/generalized-intelligence/GAAS.svg?style=for-the-badge)
![issues](https://img.shields.io/github/issues-raw/generalized-intelligence/GAAS.svg?style=for-the-badge)



![star](https://img.shields.io/github/stars/generalized-intelligence/GAAS.svg?style=social)

![twitter](https://img.shields.io/twitter/follow/GAAS_ooo.svg?style=social)

![future](https://img.shields.io/badge/Let's%20go%20back%20to-the%20Future!-blue.svg)

You Can Chat and Find Support at: [![Join the chat at https://gitter.im/GAAStalk/community](https://badges.gitter.im/GAAStalk/community.svg)](https://gitter.im/GAAStalk/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Or watch the step-by-step [tutorial](https://github.com/generalized-intelligence/GAAS/blob/master/demo/tutorial_1/README.MD)

Or follow the lastest news about the project at our [Medium Publication](https://medium.com/generalized-intelligence)

扫码加入 GAAS 微信讨论群（由于现有的群人数都超过 100 人，需要先加 GAAS 管理员，由管理员发送入群邀请）：

<img width="200" height="200" src="https://s2.ax1x.com/2019/04/11/A7y6SS.jpg"/>

- [What is GAAS?](#what-is-gaas)
  * [Project Overview](#project-overview)
  * [Tutorial for GAAS:](#tutorial-for-gaas)
  * [Setup](#setup)
    + [1. Install ROS-Kinetic](#1-install-ros-kinetic)
    + [2. Install Octomap-Server and mavros](#2-install-octomap-server-and-mavros)
    + [3. Gazebo](#3-gazebo)
    + [4. PCL](#4-pcl)
    + [5. YGZ-slam](#5-ygz-slam)
    + [6. PX4](#6-px4)
  * [Sample Use Case](#sample-use-case)
  * [Development Setup](#development-setup)
  * [Meta](#meta)
  * [Special Thanks](#special-thanks)
  * [Contribute](#contribute)
    + [I just want to build an autonomous drone](#i-just-want-to-build-an-autonomous-drone)
    + [I want to contribute to the project](#i-want-to-contribute-to-the-project)
    
- [GAAS 是什么？](#gaas-是什么)
  * [项目总览](#项目总览)
  * [这里是 GAAS 的详细教程：](#这里是-gaas-的详细教程)
  * [安装](#安装)
    + [1. 安装 ROS-Kinetic](#1-安装-ros-kinetic)
    + [2. 安装 Octomap-Server 以及 mavros](#2-安装-octomap-server-以及-mavros)
    + [3. Gazebo](#3-gazebo-1)
    + [4. PCL](#4-pcl-1)
    + [5. YGZ-slam](#5-ygz-slam-1)
    + [6. PX4](#6-px4-1)
  * [使用案例](#使用案例)
  * [开发配置](#开发配置)
  * [Meta](#meta-1)
  * [特别感谢](#特别感谢)
  * [为项目做贡献](#为项目做贡献)
    + [我想造一台自动驾驶无人机](#我想造一台自动驾驶无人机)
    + [我想为项目做贡献](#我想为项目做贡献)

# What is GAAS?

<img src="https://s2.ax1x.com/2019/01/31/k1TWUU.jpg" align="right" width="300px" alt="GA">

> Generalized Autonomy Aviation System (GAAS, pronounciate as "gas") is an open source project dedicated to autonomous flight and operating control system for futuristic aerial vehicles.

> Imagine commuting with "flying car", having food and packages delivered by drones to your door. To contribute to the described future, we would like to get the two most fundamental issues out of the way: the safety of aerial vehicles, and the demand for skilled pilots. 

> Our long-term vision is to implement GAAS in passenger carrying aerial vehicles (could be "flying car"). The first step of this vision is to make Unmanned Aerial Vehicles truly "unmanned", so drones can be better utilized. We also just want to provide an easy-to-use infrastructure for programmers and engineers to program and build drone products.

> The current version of GAAS includes Simultaneous Localization and Mapping (SLAM), obstacle avoidance, navigation and flight control, supporting manned and unmanned multi-rotor drones and helicopters. Next step, we will also be supporting VTOL and eVTOL.

<p align="center">
<img src="https://s2.ax1x.com/2019/02/25/kIZ3vj.jpg" width="300px">
</p>

<p align="center">
An Example of Assembled Hardware
</p>


## Project Overview

Currently the project provides the following funcitons: automatic taking off and landing, navigation in GPS denied environment, obstacle avoidance and path planning based on stereo vision, scene recoginition, 3D model generation, object following, object detection, instance segmentation, and a number of python based API as well as a series of tutorials.

1. Details about automatic taking off and landing can be found in: software/px4_mavros_scripts;
2. Navigation in GPS denied environment can be found in: software/SLAM/ygz_slam_ros, currently we are using stereo optical flow;
3. Obstacle avoidance based on stereo vision can be found in: software/Obstacle_Map;
4. Path planning can be found in software/Navigator;
5. Scene recoginition, given an image, recover its position in terms of given environment, details can be found in algorithms/scene_retrieving;
6. 3D modeling, details can be found in algorithms/sfm;
<p align="center">
<img width="300" height="300" src="https://s2.ax1x.com/2019/04/01/Asv6at.png"/>
</p>

7. Object following, details can be found in algorithms/object_trace_tracking;

[![Object Following](https://s2.ax1x.com/2019/04/01/AsvuCT.png)](https://youtu.be/C6902HKUVR8)

8. Object detection, details can be found in algorithms/image_detection;

<p align="center">
<img width="300" height="300" src="https://s2.ax1x.com/2019/04/01/AsOyV0.jpg"/>
</p>

9. Instance segmentation, details can be found in algorithms/image_detection;

10. A list of control API based on MAVROS, and a series of tutorials can be found in GAAS/demo.

## Tutorial for GAAS:
https://gaas.gitbook.io/guide/


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

We are also very grateful for our contributors. You may be able to find them at [AUTHORS.md](https://github.com/generalized-intelligence/GAAS/blob/master/AUTHORS.md).

## Contribute
### I just want to build an autonomous drone
You have come to the right place!

If this is your first time building an autonomous aviation system, check out our [first Tutorial](https://github.com/generalized-intelligence/GAAS/tree/master/demo/tutorial_1). You will get a basic understanding of what MavROS, PX4 and Gazebo are, which are fundamental for the success of your autonomous drone.

If you are stuck with configuration, you may:
1. Google the error messages and see if someone else has solved a similar problem.
2. Visit the [Issues Page](https://github.com/generalized-intelligence/GAAS/issues) to see if others have provided solutions for a similar problem.
3. If neither Step 1 or Step 2 were able to help you, submit an issue to let the community know that you need help. 

If you are an advanced user, feel free to help others to get started, contribute by solving issues, or share with us about your project on our [Gitter group chat](https://gitter.im/GAAStalk/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge). 

### I want to contribute to the project

We are so grateful for your interest in contributing!

To start contributing, you need to become familiar with PX4 and MavROS, as well as the workflow of [GitHub](https://github.com/MarcDiethelm/contributing/blob/master/README.md). 

A good place to start is to look at the [open issues](https://github.com/generalized-intelligence/GAAS/issues). From there, you may choose one that interests you to tackle, or open an issue of your own to communicate with other developers. 

PS: One of the best ways to contribute is to help others to kick off their autonomous drone journey. Pay attention to the “Configuration” label in issues page to help others get started.
For more details, please follow [CONTRIBUTING.md](https://github.com/generalized-intelligence/GAAS/blob/master/CONTRIBUTING.md)

Il Vole

---

# GAAS 是什么？
<img src="https://s2.ax1x.com/2019/01/31/k1TWUU.jpg" align="right" width="300px" alt="GA">

> 泛化自动驾驶飞行器系统 Generalized Autonomy Aviation System（以下简称 GAAS）是一个面向未来飞行器设计的开源驾驶以及运行管理系统。

> 想象一个每天用飞行汽车出门，无人机送餐和快递的未来。我们希望为这样的未来解决一些基础性的问题：满足对安全性的高要求并消除飞行员以及飞手的需求。

> 我们第一步的目标是让无人机做到真正无人，从而能够为人类进行更多的工作。并且可以让更多程序员和工程师更简单的为无人机编写程序和制作产品。长期目标是将 GAAS 应用在载人飞行器上（也许是飞行汽车）。

> 当前版本的 GAAS 包含 SLAM、避障路径规划、飞行控制模块；用于支持无人和有人的旋翼和直升机驾驶。下一步计划支持 VTOL 和 eVTOL。

<p align="center">
<img src="https://s2.ax1x.com/2019/02/25/kIZ3vj.jpg" width="300px">
</p>

## 项目总览
当前无人机可以实现自动启飞，降落，无 GPS 环境下自主飞行，基于双目视觉的避障以及路径规划，场景重识别，3D 建模， 物体跟踪，物体检测, 语义分割等功能，同时还提供了一系列基于python的无人机控制API以及中英文教程。

1. 其中自动启飞，降落等功能实现在 software/px4_mavros_scripts；
2. 无 GPS 环境下自主飞行功能的实现在 software/SLAM/ygz_slam_ros, 目前使用的是基于双目光流的 SLAM；
3. 基于双目视觉的避障的实现在 software/Obstacle_Map；
4. 路径规划的实现在 software/Navigator；
5. 场景重识别，即给定一张图片，恢复出当前图片在给定环境中的位置，具体实现在 algorithms/scene_retrieving；
6. 3D 建模，具体实现在 algorithms/sfm；

<p align="center">
<img width="300" height="300" src="https://s2.ax1x.com/2019/04/01/Asv6at.png"/>
</p>

<p align="center">
硬件组装示例
</p>

7. 物体跟踪，具体实现在 algorithms/object_trace_tracking；

[![Object Following](https://s2.ax1x.com/2019/04/01/AsvuCT.png)](https://youtu.be/C6902HKUVR8)

8. 物体检测，具体实现在 algorithms/image_detection；

<p align="center">
<img width="300" height="300" src="https://s2.ax1x.com/2019/04/01/AsOyV0.jpg"/>
</p>

9. 语义分割，具体实现在 algorithms/image_segmentation；
10. 无人机控制 API 及中英文教程，具体在 demo 文件夹。

## 这里是 GAAS 的详细教程：
https://github.com/generalized-intelligence/GAAS/tree/master/demo

文字教程所在的 Yuque 并不是一个很好的分享平台，如果你有更合适的选择，欢迎告诉我们！

## 安装

NOTE: 目前只在 Ubuntu 16.04 以及 ros-kinetic 环境下测试通过。

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

## 特别感谢

我们的项目并不是从零开始，而是站在了过去十余年无人机行业先驱的肩膀上。非常感谢 PX4 与 Dronecode 等组织对无人机行业发展的推动，让我们今天得以有机会制作 GAAS!

同时我们也十分感谢本项目的贡献者，你们可以在 [AUTHORS.md](https://github.com/generalized-intelligence/GAAS/blob/master/AUTHORS.md) 中认识他们。

感谢地面站社区在中文互联网上为 GAAS 提供的支持。如果你也对中文无人机社区感兴趣，欢迎访问：

http://shequ.dimianzhan.com/topics

## 为项目做贡献
### 我想造一台自动驾驶无人机
这个项目正好适合你！
如果这是你第一次着手配置自动驾驶无人机的系统，可以看看我们[教程的第一课](https://github.com/generalized-intelligence/GAAS/tree/master/demo/tutorial_1)。你会得到一些关于 MavROS，PX4 和 Gazebo 的介绍，它们是无人机自主飞行的基础。

如果在环境配置中遇到了困难，你可以尝试：
1. 在泛用搜索引擎中搜索错误报告，看看是否有其他人也遇到了类似的问题。
2. 在 [Issues](https://github.com/generalized-intelligence/GAAS/issues) 页面看看社区中是否有其他人遇到了类似的问题。
3. 如果前两步都无法帮助你，你可以提交一个新的 Issue 并加上 "Configuration" 的标签，寻求社区力量的帮助。

如果你已经可以熟练的运用这套系统，你也可以在社区里帮助新手上路，解决一些 Issue，或者在微信群里将你的项目进展分享给我们。


### 我想为项目做贡献
我们非常感激您对项目做贡献的意愿。
首先，你需要对 PX4, MavROS 和 Gazebo 有一定的熟悉程度，并且熟悉 [GitHub 的工作流程](https://git-scm.com/book/zh/v2/GitHub-对项目做出贡献)。

[Open Issues](https://github.com/generalized-intelligence/GAAS/issues) 页面是一个好的开始。你可以看看有哪些已知的问题是你感兴趣的，或者你也可以新建一个 Issue 来告诉我们你的看法和有哪些想提高的地方。

另外，帮助项目运营最好的方式之一是帮助那些刚刚接触无人飞行的伙伴们快速上手这套系统。关注 Issues 页面中带有 “Configuration” 标注的 Issue 来帮助别的伙伴上手系统。具体细节请参阅 [CONTRIBUTING.md](https://github.com/generalized-intelligence/GAAS/blob/master/CONTRIBUTING.md)

Il Vole
