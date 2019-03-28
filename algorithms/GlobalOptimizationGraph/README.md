Do global optimization of position and attitude.Keep these info inside a structure,so that when SLAM or FlightController Reset its state,still do we have a stable estimation of drone.
Raw data will be input from mavros/ros SLAM node;Compile this with catkin.
Combine visual slam info,external AHRS info,flight controller attitude info,GPS info,scene retriever info and magnet info.
May be useful when dealing with flight sessions.
Still an immature thought,though.

全局优化飞行器的位置和姿态.保存这些信息以便在飞控或SLAM状态重置/重启动时仍然有稳定的位置估计.
原始信息从不同的ros node中发送(如mavros node,SLAM node等).只要实现这一消息发送即可,也可以从外部传感器中来.
编译这个包需要依赖catkin.

这里使用视觉SLAM消息,外部AHRS消息,飞控估计姿态消息,GPS消息(如果有),场景重定位消息(如果有),磁力计信息.

这个模块对创建飞行会话(一段连续任务,如一连串定位关系紧密相关的航点,或一次流畅的精准降落)会很有用.
虽然仍然是一个未成熟的想法.想法和代码都有待补充.



