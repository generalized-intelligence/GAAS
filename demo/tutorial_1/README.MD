## OFFBOARD Control and Gazebo Simulation
<a href="http://www.youtube.com/watch?feature=player_embedded&v=NrxKOEnidOo
" target="_blank"><img src="http://img.youtube.com/vi/NrxKOEnidOo/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>

## User Guide
See user guide at: https://gaas.gitbook.io/guide/

This folder contains some python scripts for controlling px4 in offboard mode in Gazebo simulation environment. It assumes you have properly installed mavros, gazebo7 and compiled px4 firmware detailed in the github front page.

To run the scripts:

## 1_px4_mavros_offboard_controller

### 1. mavros

    roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

### 2. px4 v1.8.0

    cd ~/px4/Firmware
    make posix_sitl_default gazebo

In this step, gazebo window will pop out and you will see a drone centered in the world origin. In another terminal, type:

    rostopic echo /mavros/state

you should see "connected"

### 3. px4 main loop
    
    cd 1_px4_mavros_offboard_controller

    python px4_python_mavros_mainloop.py

now you should see the drone gradually takes off to a predefined height.

### 4. simulation controller

use functions defined in commander.py to controll the drone. Or:

    cd 1_px4_mavros_offboard_controller

    python commander.py

### NOTE:

you can use :

    rostopic echo /mavros/state

to check the connection between PX4 and mavros.


-----

## OFFBOARD控制以及Gazebo仿真

详细版点此查看：https://gaas.gitbook.io/guide/

此文件夹内包含一些基于 Python 以及 Gazebo 的脚本， 脚本可以用来控制基于 PX4 的无人机在 Offboard 模式下进行指点飞行。使用脚本需要你安装了 mavros， Gazebo7 并且编译了 PX4。

如果想使用脚本：

## 1_px4_mavros_offboard_controller

### 1. mavros

    roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

### 2. px4 v1.8.0

    cd ~/px4/Firmware
    make posix_sitl_default gazebo

Gazebo 窗口会弹出， 并且你会看到无人机在世界坐标系中心，在另外一个终端， 使用：

    rostopic echo /mavros/state

你应该看到 「连接」字样。

### 3. px4 main loop
    
    cd 1_px4_mavros_offboard_controller
    python px4_python_mavros_mainloop.py
    
此时你应该看到无人机逐渐起飞到预设高度

### 4. simulation controller

使用 commander.py 中的 api 来控制无人机。

### NOTE:

你可以使用：

    rostopic echo /mavros/state

来判断 mavros 以及 PX4 的连接状态， 请确保显示结果为 "connected"。

### 其他

如果你遇到了：

  The authenticity of host 'github.com (13.229.188.59)' can't be established.
  
这个问题，可以在此查看解决方案：

https://ailearning.blog.csdn.net/article/details/88821014

感谢达摩克利斯s
  
 
