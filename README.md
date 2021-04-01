# GAAS_contrib: 向完全取代驾驶员的自主驾驶飞行汽车进发————一组稳健的 GAAS 激光雷达扩展框架!

# GAAS_contrib: Towards L5 Autonomous Flying Cars, a Robust Framework Extends GAAS with Lidars!

Extra modules of GAAS; Including some amazing lidar-based algorithms!

**本项目包含一些 GAAS的扩展模块,包括一些基于激光雷达的优秀算法实现!**

![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/algorithms/preview_imgs/gaas_algorithms_rviz_preview_20200401.png)

![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/algorithms/preview_imgs/gaas_algorithms_rqt_graph_20200401.png)


**这个项目是 GAAS----一套无人机/飞行汽车全自主飞行框架的扩展模块.**

## 目录 Index

1.使用说明 Usage

2.开启项目的原因以及作者想说的 The reason why GAAS_contrib was created and the words by the author

3.现在已完成的部分 Finished tasks by now

4.开发路线图 Roadmap

## 1.使用说明 Usage

###     1.编译工程 Build the project:

To build the project, setup all dependencies and run:

    ./build_all.sh

###     2.仿真环境 Simulation:

To start simulation, check out simulation/README.md to setup everything;

run:

    ./scripts/prepare_simulation.sh
    
to start your gazebo simulation.    

###     3.算法 GAAS_contrib_algorithms.

To run GAAS_contrib algorithms, just:

    cd algorithms
    ./run_gaas_contrib_algorithms.sh

## 2.开启项目的原因以及作者想说的 The reason why GAAS_contrib was created and the words by the author

开启此项目的原因是: GAAS基于纯视觉算法的思路,目前看来并不是很好地适应无人驾驶行业发展的现状.GAAS本身设计之初,并不是纯粹为载人自主飞行器(以下统称:飞行汽车)和货运无人机场景设计的,因此不可避免有一些问题.

The reason why this project is created is that GAAS is a fully vision-algorithms based framework which is not that robust for autonomous-driving. GAAS is not purely designed for flying cars and large cargo drones.

对于大型货运无人机和飞行汽车而言,纯视觉的技术路线缺点是:

For flying cars and large cargo drones, pure vision-based algorithms suffers from:

1.鲁棒性不够,不能针对夜间或逆光等场景有较好的适应性,飞行器高速移动时定位也容易丢失,容易引发恶性事故.**这是大型飞行器绝对不能接受的.**

Lack of robustness, especially at night or over-exposed conditions. When vehicles are flying with high speed, the localization is not stable enough, which may cause severe accidents. **(vital to large vehicles)**

2.计算量比较大.在移动设备上不容易实现实时运行.

Too much computational cost, which can hardly be deployed on mobile devices.

3.基于神经网络的方法在极端场景下容易出现事故,且故障不易复现.

Neural Network based algorithms can cause accidents in corner case, which is hard to debug and analyze.

这些问题都是载人飞行场景下不允许发生的.

These kind of problems are not allowed to occur on autonomous flying cars.

因此,**引入激光雷达目前看来是十分必要的.**

so that **lidars are vital to L5 autonomous flying cars**.

考虑备选的型号主要是:

We consider lidars like:

Livox TELE-15

Velodyne Velarray


考虑获取的点云的稠密性和大量应用的成本,固态激光雷达显然是未来更有前景的方向.

Lidars without spinning components are more economical and robust.

使用激光雷达将大大减少感知,定位模块中一些算法的复杂程度,提高稳健性;实现建图及自动降落等功能也较为容易.

By utilizing lidars the complexity of perception and localization modules can be reduced and the robustness can be improved obviously.

It is also easier to build HD-map and implement precise landing by lidars.


主要考虑添加的算法有:

Algorithms to be implemented is mainly:

1.点云匹配. ICP + Graph Optimization 算法.用于离线激光建图,在线激光 SLAM定位,激光重定位等.

Pointcloud registration. For mapping & matching.

2.点云结合视觉的运动中障碍物检测.可以解决视觉难以解决的线状物体检测问题和深度恢复问题,规划更稠密精准的可行驶区域.

Obstacle detection. Solve the problem of detecting line-shaped objects which are hard to find out by visual features, and plan a more accurated drivable space.

3.激光雷达地障检测.在未知场地备降.

Lidar terrain detection. For landing in unknown places.


模拟器仍考虑使用 Gazebo.

Still consider gazebo as the simulator.

目标平台暂定x86 i7级别处理器(如upextreme平台),用于基础算法的部署,便于开发和仿真测试; Nvidia Jetson AGX Xavier(32GB RAM,30TOPs),用于基于神经网络算法/需要 CUDA加速算法的部署;这两款平台也有充足的性能冗余,能保证算法运行的稳健性.

Target hardware platform will be something like upextreme with x86 i7 processors for developing and simulation.

Nvidia Jetson AGS Xavier may be used for deploying neural networks/cuda acceleration based algorithms.

更低端的 Jetson tx2, Raspberry pi, Odroid等性能太差,在大型飞行器场景成本对计算单元成本要求不高的场景下,不予考虑.

Chips like Jetson tx2 or Raspberry pi, Odroid will not be supported due to their extremely low performance. Flying cars and large cargo drones are not sensitive to the cost of computation units.

## 3.现在已完成的部分 Finished tasks by now

包含32线激光雷达和双目摄像头的仿真环境初步搭建.

Simulation env with 32 lines lidar and stereo camera.

32线Lidar+Stereo camera 效果如下图.
![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/simulation/preview_imgs/Velodyne_HDL32E_sim.jpg)

Livox Horizon的模拟:
![image](https://github.com/cyanine-gi/GAAS_contrib/raw/main/simulation/preview_imgs/Livox_sim.jpg)
(图像较大,请耐心等待加载)

32线旋转式Lidar建图 & NDT定位. Spinning lidar mapping and ndt matching localization.

仿真环境运行详细说明见simulation/README.md.

Checkout simulation/README.md to get more details of simulation env setup.

## 4.开发路线图 Roadmap

### 1.实现gazebo 仿真环境的搭建,包括重复扫描式激光雷达和非重复扫描式激光雷达与双目相机的多种组合方式. Gazebo simulation env construction, including spinning lidars and non-repetitive lidars and stereo cameras.

##### 1.Livox Horizon + Forward Stereo Camera --Done.

##### 2.Velodyne HDL-32 + Forward Stereo Camera --Done.

### 2.方便的部署脚本,解决GAAS从头编译安装较为复杂的问题. Some scripts to accelerate compiling and deployment.

### 3.实现一些以激光雷达(机械/固态)为主的算法,并在仿真环境中实现一键启动. Algorithms based on lidars and scripts to start everything in simulation env.

##### 1.Lidar Camera 前融合 点云投影到图像 --完成.

##### 2.Euclidean Cluster Extraction. --Done.

##### 3.Global coordinate based HD-Map building. --Done. 

##### 4.NDT Lidar Localization --Done.

##### 5.Downsampling Node --Done.

##### 6.Interactive GUI target selector in HD-Map --TODO.


---------------

## GAAS_contrib目前主要分为两个部分:算法和仿真.

### **1.仿真部分**

实现带有双目摄像头和激光雷达的飞行器模拟,将包含非重复扫描的激光雷达模拟实现,以帮助开发者在模拟器环境中仿真类似Livox mid40/Horizon一类的非重复扫描雷达.

### **2.算法部分**

负责实现基于激光雷达和摄像头的感知,建图和定位方案.

负责实现空域动态分配算法.
