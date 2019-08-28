:star: Star us on GitHub — it helps!
# Using AirSim to Simulate Aircraft Inspection by Autonomous Drones

Regular maintenance of aircraft is critical to the safety of flights. However, due to the size of the aircraft, maintenance technicians are having trouble inspecting the top of the aircraft. Since 2017, Airbus has been using drones to help with the maintenance of aircraft. The use of drones allows maintenance technicians to collect a large number of pictures of the aircraft for safety analysis.

For Chinese Version 中文版说明：https://gaas.gitbook.io/guide/case-study/zai-airsim-zhong-mo-ni-wu-ren-ji-dui-ke-ji-xun-jian

Existing drones are not suitable for such task. First, it requires pilots to operate the drone. A minor deviation from the planned flight route could damage the aircraft. Also, for effective inspection, the pictures of aircraft parts must be taken at the same angle every time. A lot of pilots are not qualified or willing to operate drones under such risky and challenging scenario. Second, traditional drones rely on GPS, but indoor aircraft storage does not have GPS reception. 

This case study shows how to use GAAS to control autonomous drones for aircraft inspection. This case study is based on a real project currently ongoing in the GAAS team. Minor details are changed for confidentiality purpose.

<p align="center">
<img src="https://github.com/generalized-intelligence/GAAS/blob/master/demo/aircraft_inspection.gif?raw=true"/>

## What is AirSim?

[AirSim](https://github.com/microsoft/AirSim) is an open-source simulator for drones, cars and more, built on Unreal Engine and developed by Microsoft. Comparing to Gazebo, AirSim has a more realistic simulated environment. AirSim supports PX4 based Hardware-In-The-Loop (HITL) and Software-In-The-Loop (SITL)。

> Like all Unreal project, AirSim projects can be packaged. However, I do not recommend you to do so under Ubuntu. Using PX4 SITL in AirSim under Ubuntu can cause the program to crash. We are working on fixing related issues.

> Also, Unreal needs around 69GB hard disk space with relatively high computational resources. If you do not want to install such a heavy software on your computer, you may choose to do the simulation in Gazebo. The configuration for Gazebo is way easier.

## Installing Unreal and AirSim
In order to run GAAS along with AirSim, we will be using Unreal for Linux.

1. First, register at [Epic Games](https://docs.unrealengine.com/en-US/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/1/index.html) and connect it to your GitHub account.

2. Compile Unreal 4.22. It may take around 69 GB hard disk space.
```bash
git clone -b 4.22 https://github.com/EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```
> **Note**：AirSim supports Unreal 4.18, but the background used in this case study is only available in Unreal 4.22.
Therefore, we will continue this case study in Unreal 4.22. 

3. Clone and compile AirSim：
```bash
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
./setup.sh
./build.sh
```
## How to launch AirSim 
1. Under Linux environment, go to the Unreal directory and run  `UnrealEngine/Engine/Binaries/Linux/UE4Editor`.
2. Now you can see the simulator interface. Under the AirSim directory `Unreal/Environments`, there is a default project named `Blocks` . Copy  `AirSim/Unreal/Enviroments/Plugins`  to `AirSim/Unreal/Enviroments/Blocks` . Then open the project in Unreal.

![Blocks Project](https://s2.ax1x.com/2019/08/21/mUkyPP.png)

## How to launch PX4 SITL
Before using PX4 in AirSim, we should install and launch PX4 SITL. Through PX4 SITL, we may use PX4 as the flight control for drones in AirSim. 

1. First, make sure that you have installed all dependencies required by PX4 as listed in the [GAAS Tutorial](https://gaas.gitbook.io/guide/).
2. Install PX4 Firmware. **Note**: AirSim requires Firmware v1.8.2.
```
bash
mkdir -p PX4
cd PX4
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.8.2 
```

3. Compile and launch SITL mode：
```bash
make posix_sitl_default none_iris
```
![Start PX4 SITL](https://s2.ax1x.com/2019/08/21/mUF59K.png)
> You can edit  `(Firmware_Path)/posix-configs/SITL/init/ekf2/iris`  to configure the settings for the drone

4. Edit [AirSim Setting](https://github.com/microsoft/AirSim/blob/master/docs/settings.md)，The file can be found at  `~/Documents/AirSim/settings.json`。
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false
    }
  }
}
```

5. In Unreal, click the Play button. The simulation is launched. You should see a drone in the simulator. 
> Press `F1` key to see a list of shortcuts. You may also open FPS to view FPS at any time. 

![Play Blocks](https://s2.ax1x.com/2019/08/21/mUkTP0.png)

6. Download [QGroundControl](http://qgroundcontrol.com/), and change the listening port, as shown below: 

![Common Link](https://s2.ax1x.com/2019/08/21/mUAlQS.png)
![Common Link UDP](https://s2.ax1x.com/2019/08/21/mUA4yD.png)

7. Now you may use QGC to control the basic movements of the drone.

## How to use GAAS to allow the drone to fly autonomously

1. GAAS uses MAVROS to communicate with PX4. First, we need to launch MAVROS. Please see [GAAS Tutorial EP01](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/wu-ren-ji-zi-dong-jia-shi-xi-lie-offboard-kong-zhi-yi-ji-gazebo-fang-zhen). We recommend using Method 2 for the installation of MAVROS. 

2. PX4 uses UDP ports for communication, so we need to change the MAVROS `fcu_url`  configuration. Start a new launch file with:
```bash
cd (MAVROS_PATH)/mavros/launch
cp px4.launch px4_airsim.launch
```
3. Change the fifth line `<arg name="fcu_url" default="/dev/ttyACM0:57600" />`  to `<arg name="fcu_url" default="udp://:14540@localhost:14557" />`。

4. Launch MAVROS：
```bash
roslaunch mavros px4_airsim.launch
```

Now, following the [GAAS Tutorial EP01](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-e01-offboard-control-and-gazebo-simulation), you may control the drone with python scripts.

```bash
cd (GAAS_PATH)/demo/tutorial_1/1_px4_mavros_offboard_controller
python px4_mavros_run.py
```
## Set up the drone and the aircraft for the inspection
1. First, build the aircraft storage background. We recommend purchasing background materials from the [Unreal Marketplace](https://www.unrealengine.com/marketplace/zh-CN/item/09dd36e13fdf4b1592a4a09db2ec995c). Purchasing and download must be done on Epic Games on Windows. Then edit the background to your preference with the help of the [Unreal Documentation].(https://docs.unrealengine.com/en-US/index.html)。Due to IP restrictions, we cannot share our Project file with everyone. If you encountered problems with building the background, we welcome you to ask your questions at AirSim repo or Unreal Forum. 

![Airplane Project](https://s2.ax1x.com/2019/08/21/mUESmQ.png)

2. For the drone to fly without GPS, let's turn off GPS altogether. First,  in PX4 SITL iris, change the `vision position fusion` in `EKF2_AID_MASK` to `8`. Then, edit the `(Firmware_Path)/posix-configs/SITL/init/ekf2/iris`  to change  `EKF2_AID_MASK` from 1 to 8.

![QGC](https://s2.ax1x.com/2019/08/21/mUEU7d.png)

3. Now we need to set up the AirSim configuration to include the stereo camera. We have provided the configuration file in GAAS.
```python
git pull origin master
cp (GAAS_PATH)/simulator/AirSim/settings.json ~/Documents/AirSim/
```
> In this config file, GPS has been set to `false,` and the stereo camera is included.

4. After the above steps, let's click the Play button to start the simulation. However, without GPS, the drone will not be able to take off. We need to turn on SLAM to allow the drone to operate in GPS-deficient environment. 


5. We have set up a stereo camera with a baseline of 12cm in the configuration file. It is the same setup as GAAS SLAM. Before running SLAM algorithms, we need to publish the camera data from the simulator to ROS. Install the Python Library for Airsim and then under GAAS directory, `simulator/AirSim` , run `drone_image_ros.py`. 

6. Next, follow [GAAS Tutorial EP03](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-part-3-using-slam-in-gps-denied-environment-for-position-estimation) to use GAAS for inspection of aircraft.

![Airplane Fly](https://s2.ax1x.com/2019/08/21/mUEcng.png)

> Note: When using SLAM, go to ` Edit >> Editor Preference` in Unreal to turn off the `Use Less CPU when in Background` , otherwise the frame rate will be affected.
