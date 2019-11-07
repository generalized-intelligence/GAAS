## 2019.04.23

1. Modified coordinate frames for drone body frame and world frame according to rep-103 and rep-105. Using FLU for body frame and ENU for world reference frame.
2. Modified reference frame figures used in tutorials part 1.


## 2019.11.07

1. Add Global_Optimization_Graph (GOG), tested on GPS + SLAM fusion with constant GPS or intermittent GPS (you need to mannually tune GPS covariance to gain a better performance in each case). Code refactoring is needed in the next few updates and large amount of field tests is also required. 
