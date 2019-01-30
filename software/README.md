软件部分的主要目标是实现一套易用的默认配置，同时满足可定制需求。
出于兼容性的考虑，支持的传感器是双目照相机。主动测距传感器（RGBD相机，激光雷达等）暂时不被考虑。
软件部分包含以下几个模块：
1.SLAM
2.Obstacle Map
3.Dynamic Target（待补充）
4.Local Planner/Global Planner（待补充）

其中每个模块会包含一个默认实现。如需替换其中的模块，需要完成原本的ros topic/service的替换。所用到的ros topic/service 如下所示。
飞控部分暂定使用px4作为默认实现。这部分暂不提供可选。

1.SLAM:
    subscribe:
                /multisense_sl/camera/left/image_raw
                /multisense_sl/camera/right/image_raw
		
    publish:
                /ygz_odom_marker
                /mavros/vision_pose/pose
                /ygz_obstacle_pub
2.Obstacle Map
    subscribe:
                /multisense_sl/camera/left/image_raw
                /multisense_sl/camera/right/image_raw
    publish:
                /cloud_in
3.Dynamic Target（待补充）

4.Local Planner/Global Planner(待补充)


