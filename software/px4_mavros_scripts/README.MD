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
use functions defined in commander.py to controll the drone.


### NOTE:

you can use :

    rostopic echo /mavros/state

to check the connection between PX4 and mavros.
