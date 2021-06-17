<a href="">
    <img src="https://s2.ax1x.com/2019/01/31/k1TWUU.jpg" alt="gaas logo" title="gaas" align="right" height="100" />
</a>

# Generalized Autonomy Aviation System

:star: Star us on GitHub — it helps!      


# We have just completed the development of version 2.0 and will open source it soon :) 

# 我们刚刚完成了 2.0 版本的开发，即将开源:)

  [![Join Facebook Group at https://www.facebook.com/groups/300340454189266/?ref=bookmarks](https://img.shields.io/badge/Group-Facebook-blue.svg)](https://www.facebook.com/groups/300340454189266/?ref=bookmarks) [![twitter](https://img.shields.io/twitter/follow/GAAS_dev.svg?style=social)](https://twitter.com/GAAS_dev)   [![Follow Medium at https://medium.com/generalized-intelligence](https://img.shields.io/badge/Medium-Blogs-black.svg)](https://medium.com/generalized-intelligence)
  
  
[<img src = "https://s2.ax1x.com/2019/11/07/Mkwj81.png">](https://github.com/generalized-intelligence/GAAS/releases/tag/v0.7)
---


[<img width="280" src="https://s2.ax1x.com/2019/06/20/Vvmsld.png"/>](https://bit.ly/2Ky8VbV)
<img src="https://s2.ax1x.com/2019/09/20/nXrXjK.png">

# What's new:

## Towards L5 Autonomous Flying Car: GAAS_contrib, A Robust Framework Extends GAAS with Lidars.

[GAAS_contrib](https://github.com/cyanine-gi/GAAS_contrib) supports some lidar-based algorithms for simulation. Now you can simulate Velodyne HDL-32 and **Livox Horizon** (a non-repeatitive lidar only cost you 800$!) with gazebo. Lidar-based perception, mapping and localization will be added, just like what happened to autonomous vehicles.


Simulation of HDL-32:
![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/simulation/preview_imgs/Velodyne_HDL32E_sim.jpg)

Simulation of Livox Horizon:
![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/simulation/preview_imgs/Livox_sim.jpg)


- [What is GAAS?](#what-is-gaas)
  * [Tutorial for GAAS](#tutorial-for-gaas)
  * [Installation](#installation)
  * [Overview](#overview)
  * [Contribute](#contribute)
  * [Meta](#meta)
  * [Special Thanks](#special-thanks)
  * [中文 readme.md](https://github.com/generalized-intelligence/GAAS/blob/master/readme_zh.md)

# What is GAAS?

<img src="https://s2.ax1x.com/2019/02/25/kIZ3vj.jpg" align="right" width="300px" alt="hardware">

> GAAS (Generalized Autonomy Aviation System) is an open source autonomous aviation software platform, designed for fully autonomous drones and flying cars. GAAS was built to provide a common infrastructure for computer-vision based drone intelligence. In the long term, GAAS aims to accelerate the coming of autonomous VTOLs. Being a BSD-licensed product, GAAS makes it easy for enterprises, researches, and drone enthusiasts to modify the code to suit specific use cases. 

> Our long-term vision is to implement GAAS in autonomous passenger carrying VTOLs (or "flying cars"). The first step of this vision is to make Unmanned Aerial Vehicles truly "unmanned", and thus make drones ubiquitous. We currently support manned and unmanned multi-rotor drones and helicopters. Our next step is to support VTOLs and eVTOLs.

## Tutorial for GAAS
See the [repo](https://github.com/generalized-intelligence/GAAS/tree/master/demo) and the [documentation](https://gaas.gitbook.io/guide/)


## Installation
Please see [Setup.md](https://github.com/generalized-intelligence/GAAS/blob/master/Setup.md)

We also have a mirror form developers to install GAAS, [click here to download](https://gaas.gitbook.io/guide/handy-tools/v0.7-release-mirror-x86).

Trust me you won't want to install every dependencies by yourself:)


## Overview
Currently the project provides the following ten funcitons, some of which may need to be further optimized: 

NOTE: This is a beta version of the software. Please re-ensure the stability of each feature before implementing on real drones.

<p align="center">
<img src="https://github.com/generalized-intelligence/GAAS/blob/master/demo/gaaspole.gif"/>
    
    VISION BASED POLE AVOIDANCE BY GAAS
</p>

1. Details about automatic taking off and landing can be found in: ```software/px4_mavros_scripts```;
2. Navigation in GPS denied environment can be found in: ```software/SLAM/ygz_slam_ros```, currently we are using stereo optical flow;
3. Obstacle avoidance based on stereo vision can be found in: ```software/Obstacle_Map```;
4. Path planning can be found in ```software/Navigator```;
5. Scene recoginition, given an image, recover its position in terms of given environment, details can be found in algorithms/scene_retrieving;
6. 3D modeling, details can be found in ```algorithms/sfm```;
7. [Object tracking](https://youtu.be/C6902HKUVR8), details can be found in ```algorithms/object_trace_tracking```;
8. Object detection, details can be found in ```algorithms/image_detection```;
9. Instance segmentation, details can be found in ```algorithms/image_detection```;
10. A list of control API based on MAVROS, and a series of tutorials can be found in ```GAAS/demo```;
11. A list of hardware that we use is at ```GAAS/hardware```.

<p align="center">
<img src="https://s2.ax1x.com/2019/11/07/MkB8Te.png"/>
</p>

## Contribute
### I just want to build an autonomous drone
You have come to the right place!

If this is your first time building an autonomous aviation system, check out our [first Tutorial](https://github.com/generalized-intelligence/GAAS/tree/master/demo/tutorial_1). You will get a basic understanding of what MavROS, PX4 and Gazebo are, which are fundamental for the success of your autonomous drone.

If you are stuck with configuration, you may:
1. Google the error messages and see if someone else has solved a similar problem.
2. Visit the [Issues Page](https://github.com/generalized-intelligence/GAAS/issues) to see if others have provided solutions for a similar problem.
3. If neither Step 1 or Step 2 were able to help you, submit an issue to let the community know that you need help. 

If you are an advanced user, feel free to help others to get started, contribute by solving issues, or share with us about your project on our [Gitter group chat](https://gitter.im/GAAStalk/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge). 

### I want to contribute

We are so grateful for your interest in contributing!

To start contributing, you need to become familiar with PX4 and MavROS, as well as the workflow of [GitHub](https://github.com/MarcDiethelm/contributing/blob/master/README.md). 

A good place to start is to look at the [open issues](https://github.com/generalized-intelligence/GAAS/issues). From there, you may choose one that interests you to tackle, or open an issue of your own to communicate with other developers. 

PS: One of the best ways to contribute is to help others to kick off their autonomous drone journey. Pay attention to the “Configuration” label in issues page to help others get started.
For more details, please follow [CONTRIBUTING.md](https://github.com/generalized-intelligence/GAAS/blob/master/CONTRIBUTING.md)

## Meta

Project initialized by Generalized Intelligence

Distributed under the BSD 3-Clause license. See ``LICENSE`` for more information.

## Special Thanks

It is worth mentioning that we did not build everything from scratch, but on top of the solid foundations built by pioneers in the field. We would like to thank communities such as [PX4](https://px4.io) and [Dronecode](https://www.dronecode.org) for constantly pushing the industry foward. What they have built are what allowed us to build GAAS!

Also, thanks for the support from https://thone.io

We are also very grateful for our contributors. You may be able to find them at [AUTHORS.md](https://github.com/generalized-intelligence/GAAS/blob/master/AUTHORS.md).

Il Vole
