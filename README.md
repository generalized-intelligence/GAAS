<a href="">
    <img src="https://s2.ax1x.com/2019/01/31/k1TWUU.jpg" alt="gaas logo" title="gaas" align="right" height="100" />
</a>

# Generalized Autonomy Aviation System

:star: Star us on GitHub — it helps!

[![Join the chat at https://gitter.im/GAAStalk/community](https://badges.gitter.im/GAAStalk/community.svg)](https://gitter.im/GAAStalk/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
  [![Join Facebook Group at https://www.facebook.com/groups/300340454189266/?ref=bookmarks](https://img.shields.io/badge/Group-Facebook-blue.svg)](https://www.facebook.com/groups/300340454189266/?ref=bookmarks) [![twitter](https://img.shields.io/twitter/follow/GAAS_ooo.svg?style=social)](https://twitter.com/GAAS_ooo)   [![Follow Medium at https://medium.com/generalized-intelligence](https://img.shields.io/badge/Medium-Blogs-black.svg)](https://medium.com/generalized-intelligence)
  
[<img src = "https://s2.ax1x.com/2019/08/16/mZt6n1.jpg">](https://forum.gaas.dev)
=======
  <img width="900" src = "https://s2.ax1x.com/2019/06/14/V4JkLD.png">

[<img src="https://s2.ax1x.com/2019/06/21/Vz7bNT.png">](https://www.facebook.com/groups/300340454189266/?ref=bookmarks)[<img src="https://s2.ax1x.com/2019/06/21/Vz7sBt.png"/>](https://bit.ly/2Ky8VbV)[<img src="https://s2.ax1x.com/2019/06/21/VzHA8e.png"/>](https://discord.gg/cUxYsRc)
<img src="https://s2.ax1x.com/2019/06/21/VzI4j1.png">

[<img width="280" src = "https://s2.ax1x.com/2019/06/20/Vvmy6A.png" >](https://www.facebook.com/groups/300340454189266/?ref=bookmarks)[<img width="280" src="https://s2.ax1x.com/2019/06/20/Vvmsld.png"/>](https://bit.ly/2Ky8VbV)[<img width="280" src="https://s2.ax1x.com/2019/06/20/Vvm6OI.png"/>](https://discord.gg/cUxYsRc)
<img src="https://s2.ax1x.com/2019/06/20/Vvmgmt.png">

- [What is GAAS?](#what-is-gaas)
  * [Tutorial for GAAS](#tutorial-for-gaas)
  * [Installation](#installation)
  * [Overview](#overview)
  * [Contribute](#contribute)
  * [Meta](#meta)
  * [Special Thanks](#special-thanks)
- [GAAS 是什么？](#gaas-是什么)
  * [使用教程：](#使用教程)
  * [安装](#安装)
  * [项目总览](#项目总览)
  * [相关硬件](#相关硬件)
  * [为项目做贡献](#为项目做贡献)
  * [其它](#其它)
  * [特别感谢](#特别感谢)


# What is GAAS?

<img src="https://s2.ax1x.com/2019/02/25/kIZ3vj.jpg" align="right" width="300px" alt="hardware">

> GAAS (Generalized Autonomy Aviation System) is an open source software platform for autonomous drones and VTOLs. GAAS was built to provide a common infrastructure for computer-vision based drone intelligence. In the long term, GAAS aims to accelerate the coming of autonomous VTOLs. Being a BSD-licensed product, GAAS makes it easy for enterprises, researches, and drone enthusiasts to modify the code to suit specific use cases. 

> Our long-term vision is to implement GAAS in autonomous passenger carrying VTOLs (or "flying cars"). The first step of this vision is to make Unmanned Aerial Vehicles truly "unmanned", and thus make drones ubiquitous. We currently support manned and unmanned multi-rotor drones and helicopters. Our next step is to support VTOLs and eVTOLs.

## Tutorial for GAAS
See the [repo](https://github.com/generalized-intelligence/GAAS/tree/master/demo) and the [documentation](https://gaas.gitbook.io/guide/)


## Installation
Please see [Setup.md](https://github.com/generalized-intelligence/GAAS/blob/master/Setup.md)


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
<img src="https://s2.ax1x.com/2019/05/17/Eq4TBD.png"/>
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

We are also very grateful for our contributors. You may be able to find them at [AUTHORS.md](https://github.com/generalized-intelligence/GAAS/blob/master/AUTHORS.md).

Il Vole

---

# GAAS 是什么？
<img src="https://s2.ax1x.com/2019/02/25/kIZ3vj.jpg" align="right" width="300px" alt="hardware">

> GAAS (Generalized Autonomy Aviation System) 是一套开源的无人机自主飞行软件平台。GAAS 致力于为无人机智能应用提供一个泛用的开发架构，以此加速自动驾驶载人 VTOL 的到来。作为一个受 BSD 协议保护的项目，任何企业、研究人员、无人机爱好者都可以合法合规地改动我们的代码来满足其客制化的需求。

> 我们第一步的目标是让无人机做到真正无人，从而能够为人类进行更多的工作，并且可以让更多程序员和工程师更简单的为无人机编写程序和制作产品。长期目标是将 GAAS 应用在载人 VTOL 上（也许是飞行汽车）。我们现在支持无人和有人的旋翼和直升机驾驶。下一步计划支持 VTOL 和 eVTOL。

## 使用教程：
详情请见[教程课件](https://github.com/generalized-intelligence/GAAS/tree/master/demo)和[教程文档](https://gaas.gitbook.io/guide/)

## 安装
参见 [Setup.md](https://github.com/generalized-intelligence/GAAS/blob/master/Setup.md)

## 项目总览
当前 GAAS 可为无人机提供以下十大功能，其中一些功能仍有待优化：

注意：GAAS beta 版部分功能尚不稳定，请在模拟器中确保稳定性后再在实机操作。

<p align="center">
<img src="https://github.com/generalized-intelligence/GAAS/blob/master/demo/gaaspole.gif"/>
    
    无人机纯视觉双目躲避室内细杆（无光流）
</p>

1. 其中自动起飞，降落等功能实现在 ```software/px4_mavros_scripts```；
2. 无 GPS 环境下自主飞行功能的实现在 ```software/SLAM/ygz_slam_ros```, 目前使用的是基于双目光流的 SLAM；
3. 基于双目视觉的避障的实现在 ```software/Obstacle_Map```；
4. 路径规划的实现在 ```software/Navigator```；
5. 场景重识别，即给定一张图片，恢复出当前图片在给定环境中的位置，具体实现在 ```algorithms/scene_retrieving```；
6. 3D 建模，具体实现在 ```algorithms/sfm```；
7. [物体跟踪](https://youtu.be/C6902HKUVR8)，具体实现在 ```algorithms/object_trace_tracking```；
8. 物体检测，具体实现在 ```algorithms/image_detection```；
9. 语义分割，具体实现在 ```algorithms/image_segmentation```；
10. 无人机控制 API 及中英文教程，具体在 ```GAAS/demo``` 文件夹。
我们所用的硬件清单在 ```GAAS/hardware```

<p align="center">
<img src="https://s2.ax1x.com/2019/05/17/EqXAdx.png"/>
</p>

## 相关硬件
我们的项目完全开源，你可以用任何你希望使用的开发方式来开发 GAAS。

但如果你希望快速上手，直接开始写代码的话，你也可以购买 GAAS 团队内部自己使用的开发套件：

https://item.taobao.com/item.htm?id=591140560551

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

## 其它

泛化智能 Generalized Intelligence 出品。
本项目受 BSD 3-Clause 协议保护。点击``LICENSE`` 了解更多

## 特别感谢

我们的项目并不是从零开始，而是站在了过去十余年无人机行业先驱的肩膀上。非常感谢 PX4 与 Dronecode 等组织对无人机行业发展的推动，让我们今天得以有机会制作 GAAS!

同时我们也十分感谢本项目的贡献者，你们可以在 [AUTHORS.md](https://github.com/generalized-intelligence/GAAS/blob/master/AUTHORS.md) 中认识他们。

感谢地面站社区在中文互联网上为 GAAS 提供的支持。如果你也对中文无人机社区感兴趣，欢迎访问：

http://shequ.dimianzhan.com/topics

Il Vole
