![GAAS Demo](https://s2.ax1x.com/2019/01/31/k16ojg.png)

The primary goal of the software is to provide an easy-to-use default configuration, which can also be altered for more advanced demands.

Considering the potential complication caused by compatibility of various hardware, the supported sensors are stereo cameras. Proximity sensors (RGBD cameras and LiDAR) are not being considered for now.

The software contains the following modules:

    1.SLAM
    2.Obstacle Map
    3.Dynamic Target（To Be Continued）
    4.Local Planner/Global Planner: A* implementation, requires improvement


Each module contains a default implementation, and you can alter for other implementations. If you would like to replace any of the modules, the original ros topic/service needs to be replaced. The corresponding ros topics/service is as the following.
PX4 is used as default and flight control, and is the only option as of now.

## 1.SLAM:

    subscribe:
                /multisense_sl/camera/left/image_raw
                /multisense_sl/camera/right/image_raw
		
    publish:
                /ygz_odom_marker
                /mavros/vision_pose/pose
                /ygz_obstacle_pub
## 2.Obstacle Map
    subscribe:
                /multisense_sl/camera/left/image_raw
                /multisense_sl/camera/right/image_raw
    publish:
                /cloud_in
		
## 3.Dynamic Target（To Be Continued）

## 4.Local Planner/Global Planner（To Be Continued）

--------------------------------------------------------------------

软件部分的主要目标是实现一套易用的默认配置，同时满足可定制需求。

出于兼容性的考虑，支持的传感器是双目照相机。主动测距传感器（RGBD 相机，激光雷达等）暂时不被考虑。

软件部分包含以下几个模块：

    1.SLAM
    2.Obstacle Map
    3.Dynamic Target（待补充）
    4.Local Planner/Global Planner（待补充）

其中每个模块会包含一个默认实现。如需替换其中的模块，需要完成原本的 ros topic/service 的替换。所用到的 ros topic/service 如下所示。
飞控部分暂定使用 PX4 作为默认实现。这部分暂不提供可选。

## 1.SLAM:

    subscribe:
                /multisense_sl/camera/left/image_raw
                /multisense_sl/camera/right/image_raw
		
    publish:
                /ygz_odom_marker
                /mavros/vision_pose/pose
                /ygz_obstacle_pub
## 2.Obstacle Map
    subscribe:
                /multisense_sl/camera/left/image_raw
                /multisense_sl/camera/right/image_raw
    publish:
                /cloud_in
		
## 3.Dynamic Target（待补充）

## 4.Local Planner/Global Planner(待补充)


