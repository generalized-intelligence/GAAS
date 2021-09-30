# Flight State Management

# 飞行阶段管理.

The main functionality of this module is to control multiple map switching and localization method( GPS-AHRS, lidar_localization) switching.

主要功能是控制多地图加载调用和导航模式切换.


Example pipeline:

系统运行完整流程:


Load all maps  ->  switch to map1  ->  start lidar localization  ->  take off  ->  switch to gps-ahrs localization  ->  cruising  ->  switch to map2  ->  switch to lidar localization  ->  land

读取所有地图 切换起降点1地图 Lidar定位 起飞 切换GPS定位 巡航 切换起降点地图2 GPS定位切换到Lidar定位 降落


So this module shall support the transformation of these states and control other modules with proper gaas_srvs.

因此这个模块要通过调用适当的gaas_srvs 支持对其他模块的状态转换控制.





