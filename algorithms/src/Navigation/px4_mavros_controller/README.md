px4_mavros_controller 模块接替以前GAAS中的px4_mavros_run.py.

启动后,先尝试进position模式,然后原地进offboard模式.

这个模块支持两种模式的运动:相对机体当前位置和相对地图的map NWU参考系.
移动飞行器:
向/gaas/navigation/target_position 发布PoseStamped目标位置(仅position.xyz生效),消息的header.frame_id可以是lidar或map,分别表示使用机体FLU和地图NWU坐标系.

旋转朝向角:
向/gaas/navigation/target_enu_yaw 发送float32(rad)型朝向角.对应角度: E:0.0  N:0.5 * pi W: pi S: 1.5 * pi


以机体当前位置作参考,不需要启动定位模块;如果以map为参考坐标输入,则需要启动localization部分的launch文件.


scripts里有一个脚本test_mavros_controller.py可用于简单测试.如果飞行器朝向东,则鸟瞰图轨迹应该是:

          E

   2      1
        
        
        
N  4      0(3)(6)



          5
