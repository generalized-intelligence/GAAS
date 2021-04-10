# GAAS_contrib 使用文档

### GAAS_contirb 分为仿真和算法两大部分,分别在simulation和algorithms两个目录中.

### 两个部分分别有自己的catkin workspace进行构建.

  可以使用构建脚本

    ./build_all.sh
    
  进行构建.

## Simulation模块的使用:

   **初次使用前必须配置simulation部分！**配置和使用说明见 simulation/README.md

## Algorithms中有以下几个模块:

### 1.Perception

  飞行器附近障碍物感知.

### 2.LocalizationAndMapping

  飞行所需的高精地图建图和飞行器在高精地图中的定位.

### 3.Navigation

  路径规划,与飞控连接并在地图坐标系和机体坐标系发送运动命令.

### 4.Preprocessing

  传感器数据预处理.


## GAAS_contrib的部署

### 准备工作

  要运行GAAS_contrib的所有算法,需要:

  1.建图 使用Lego-LOAM等框架完成高精地图创建,同时保存建图所用的原始rosbag.将rosbag中存储的地图转换成pcd格式;配置好 map_config_generator.launch中的参数,然后:

    roslaunch lidar_mapping map_config_generator.launch

  2.在simulation/scripts/prepare_simulation.sh 中更改对应的地图,并换上合适的载具.

### 启动算法

  先运行:

    cd simulation&&scripts/prepare_simulation.sh

  然后启动algorithms所有在线运行的模块:

    cd algorithms && run_gaas_contrib_algorithms.sh

  即可.
