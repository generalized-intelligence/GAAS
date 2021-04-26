# GAAS_contrib algorithms

GAAS_contrib的所有自动驾驶算法软件包.

Packages of autonomous driving algorithms.

### 注意:

检查你的cuda版本,和机器的配置.如果不需要使用cuda加速的ndt,或者不使用默认的cuda arch(使用不适当的cuda arch配置可能会引起ndt定位node崩溃),需要手动修改lidar_localization的CMakeLists.txt.

### Pay attention:

Check your cuda version and GPU of your computing platform. If you wanna disable cuda or change the version of cuda arch(which must be set correctly or may cause your ndt_localization node crash), do edit your CMakeLists.txt of lidar_localization manually!


