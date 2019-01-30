![GA Logo](https://s2.ax1x.com/2019/01/30/klnmHP.jpg)
# Generalized Autonomy Aviation System
# GAAS
> 泛化自动驾驶飞行器系统 Generalized Autonomy Aviation System（以下简称 GAAS）是一个面向未来飞行器设计的开源驾驶以及运行管理系统。

> 想象一个每天用飞行汽车出门，无人机送餐和快递的未来。我们希望为这样的未来解决一些基础性的问题：满足对安全性的高要求并消除飞行员以及飞手的需求。

> 我们第一步的目标是让无人机做到真正无人，从而能够为人类进行更多的工作。并且可以让更多程序员和工程师更简单的为无人机编写程序和制作产品。长期目标是将 GAAS 应用在载人飞行器上（也许是飞行汽车）。

>当前版本的 GAAS 包含 SLAM、避障路径规划、飞行控制模块；用于支持无人和有人的旋翼和直升机驾驶。下一步计划支持 VTOL 和 eVTOL。


## Setup
NOTE: 目前只在ubuntu 16.04 以及 ros-kinetic 环境下测试通过。

1.安装 ROS-Kinetic

    建议安装ros-kinetic-desktop-full; 可按照此地址进行安装：http://wiki.ros.org/kinetic/Installation/Ubuntu.

2.安装 Octomap-Server 以及mavros

    cd ~/catkin_ws/src
    git clone https://github.com/OctoMap/octomap_mapping.git
    git clone https://github.com/mavlink/mavros.git
    cd ..
    catkin build
    source ~/catkin_ws/devel/setup.bash

3. Gazebo

如果您在第一步安装了ros-kinetic-desktop-full, 您已经具有了gazebo7；, 接下来请安装对应ros plugin:

    sudo apt install ros-kinetic-gazebo-*

如果您想使用其他版本的gazebo, 请参考：http://gazebosim.org/, 并安装对应版本的ros plugin:

    sudo apt install ros-kinetic-<your gazebo version>-*


4. PCL

    请参考此链接安装：http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php


3. YGZ-slam

    请参考此链接安装：:https://github.com/gaoxiang12/ygz-stereo-inertial


4. PX4

目前只在px4 v1.8.0测试通过

    mkdir ~/px4 && cd ~/px4
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    git checkout v1.8.0
    

## Usage example


1.带避障，不依赖 GPS 的路径规划指点飞行。

	step<1> 如果是室内环境使用或需要高精度飞行，ROS launch file 加入 SLAM；否则跳过此步骤。
  
	step<2> 对于室外环境，配置 GPS 轨迹(使用地图)。对于室内部分，在 3D 模型中配置轨迹。
  
	step<3> ROS launch file 加入避障部分。
  
	step<4> 启动飞行器，等待 PX4 自检通过。
  
	step<5> 启动 Mavros。
  
	step<6> 开始执行任务。

即将推出：

带避障的视觉跟踪任务

视觉精准降落

区域自动探索建模

多机协同执行任务


## Development setup

Setup 部分中所有想要更改的组件源码安装。

在 ROS 的对应接口进行替换即可。



## Meta

泛化智能 Generalized Intelligence 出品。


Distributed under the BSD 3-Clause license. See ``LICENSE`` for more information.

[GAAS GitHub](https://github.com/generalized-intelligence/GAAS)

## Contribute

请参阅 CONTRIBUTE.md

Il Vole

