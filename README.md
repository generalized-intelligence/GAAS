# Generalized Autonomy Aviation System
# GAAS
> 泛化自动驾驶飞行器系统 Generalized Autonomy Aviation System（以下简称 GAAS）是一个面向未来飞行器设计的开源驾驶以及运行管理系统。

> 想象一个每天用飞行汽车出门，无人机送餐和快递的未来。我们希望为这样的未来解决一些基础性的问题：满足对安全性的高要求并消除飞行员以及飞手的需求。

> 我们第一步的目标是让无人机做到真正无人，从而能够为人类进行更多的工作。并且可以让更多程序员和工程师更简单的为无人机编写程序和制作产品。长期目标是将 GAAS 应用在载人飞行器上（也许是飞行汽车）。

>当前版本的 GAAS 包含 SLAM、避障路径规划、飞行控制模块；用于支持无人和有人的旋翼和直升机驾驶。下一步计划支持 VTOL 和 eVTOL。




![GA Logo](https://s2.ax1x.com/2019/01/29/kQa7X6.jpg)



## Setup

1.安装 ROS-Kinetic 或更高版本ros。

2.安装 Octomap-Server、Gazebo、PCL.

3.依照 YGZ-slam 依赖项进行配置。项目地址:https://github.com/gaoxiang12/ygz-stereo-inertial

4.如果使用模拟器：
	下载 PX4 源码。地址：https://github.com/PX4/Firmware 编译。
	使用 px4_sitl_posix。
  
  如果使用实机进行模拟：可以直接下载 PX4 固件进行烧录。

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

## Contributing

1. Fork it (<https://github.com/generalized-intelligence/GAAS/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

