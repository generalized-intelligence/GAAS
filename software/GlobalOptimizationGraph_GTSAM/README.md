# Global Optimization Graph: A Loosely-Coupled Sensor Fusion Framework Based On GTSAM

### Constant GPS + SLAM fusion

The SLAM and GPS are both constant at all time, so GOG will try to fuse the pose information from these two sources.
    
<img src = "https://s2.ax1x.com/2019/11/07/MF5Ru4.png">


### Intermittent GPS + SLAM fusion

The SLAM is assumed to be constant at all time, while GPS is intermittent, being available for 5 seconds of a 10 seconds cycle.  The red number, 1 or 0, represents whether GPS is available, 1 represents GPS is lost while 0 means 
GPS is not lost. 

The blue line (the top one), is the output of GOG, which trys to fuse information from GPS and SLAM. When GPS is lost, GOG will continue to give estimation of the state, but as soon as GPS comes back, it updates the state to the corresponding pose provided by the GPS(the gap is the error of wrong SLAM estimation while GPS is lost).
    
The drone is controlled to fly a rectangle, which can be represented by the green intermittent GPS path. The bottom red line is the output of SLAM, and you can find it has deviated from the GPS path greatly, but the output of GOG(the top line) choose to believe GPS more (because it has a lower covariance), and it is also capable of "filling the gap" when GPS is lost.

You can choose to create your own test bag using a script provided in ./scripts/create_bag.py, and while recording, you need to turn on a SLAM and GPS. Remember the frame of SLAM should be in ENU.
    
<img src = "https://s2.ax1x.com/2019/11/07/MF5HgO.png">



## How To USE
    
This project is based on GTSAM, Eigen, Opencv, so you will need to have them installed and built. After installation of these packages, in a terminal:
    
    mkdir build
    cd build
    cmake ..
    make
    cd ..
    ./bin/GlobalOptimizationGraph_main config.yaml
    
In another terminal, you can play a bag containing the topic of a SLAM and the topic of GPS.
Modify line 205 and 206 in ROS_IO_MANAGER.h to change SLAM and GPS topics, we will move them to a config file later.

The output of SLAM, GPS and GOG results are saved to files in folder ./results, use visualize.py to visualize the results.

 ## Extra Words
 
     This work remains to be broadly tested.
    
    
