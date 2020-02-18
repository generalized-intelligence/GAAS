# Scene Retrieving

Enabling the drone to recover its pose in a predefined map. By recovering its pose in a map, the drone can eliminate its accumulated pose error. At the same time, the drone is capable of flying to a selected target in RVIZ.

### How To Use

1. in ygz_slam, after building:
   
   sh makescene.sh
   
   at the same time, use controller to control the drone to fly aroung an environment, and by doing this, the environment, consisting of points and key poses, can be serialized in a file ended with ".scene", copy generated folder "image" to the root of
scene_retriving. Then continue to the next step.

2. in a terminal:
    
    sh generate.sh
    
3. clone and build the following package for selecting building corners:

    https://github.com/Doodle1106/visualization_tutorials
    
If you open up an RVIZ window, if you built the package successfully, you can find the "Plant Flag" button as shown in the image.

<img src = "https://s2.ax1x.com/2019/11/14/MtjydA.png">

    
4. after building:
    
    sh point_and_fly.sh
    
4. at the same time, open a RVIZ window, use "Plant Flag" to select 4 building corners and door position to provide a prior for path finding problem( Working in Progress), then takeoff the drone, and select a position using "2D NAV Goal". The drone will fly to the selected target, and the green marker appeared in RVIZ are the recovered pose.

NOTE: The building corner selecting process should follow the number:

<img src = "https://s2.ax1x.com/2019/11/14/MtxKuF.png">
  
  and do not forget to select the position of the door.



