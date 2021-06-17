# GAAS Mirror for TX2 beta

## What is GAAS Mirror for TX2 beta?
GAAS Mirror for TX2 beta provides a one-step installation tool to set up your TX2 with the GAAS framework. The mirror includes installation of Ubuntu 16.04, ROS, other GAAS dependencies, GAAS git repo, compiled SLAM module, MYNTAI SDK.

#### [Download Now](https://1drv.ms/u/s!Ai0N8dl2SJ8HiFUaAsS9Jvf3i4UH?e=dafBBh)
You need to sign in a Microsoft account to download the Mirror as it is shared through One Drive. The mirror is about 19GB.

## How to use GAAS Mirror for TX2 beta?

### Step Zero: Verify File Integrity
MD5: 24151c13609338ffb33786a0366242a9

SHA1: 376a0507e99fe1edd91fe3a9e01f55d00db2609e

### Step One: Install Nvidia Jetpack
Download NVIDIA Jetpack from the [NVIDIA website](https://developer.nvidia.com/embedded/jetpack-archive). You may need to register for an account. Note that GAAS is running on Ubuntu 16.04 and thus the highest version we can use for JetPack is NVIDIA Jetpack3.3.


Download and run JetPack 3.3, as shown below:

<img src="https://s2.ax1x.com/2019/09/16/nWetII.png" width=500>


Choose installation directory:

<img src="https://s2.ax1x.com/2019/09/16/nWeUit.png" width=500>


Choose Jetson TX2:

<img src="https://s2.ax1x.com/2019/09/16/nWerLQ.png" width=500>


Choose the desired package. Please note only install the Linux for TX2 while leaving the other options as "no action":

<img src="https://s2.ax1x.com/2019/09/16/nWeJZd.png" width=500>


Accept all Terms and Conditions and begin downloading:

<img src="https://s2.ax1x.com/2019/09/16/nWe8qH.png" width=500>


Wait for the download and installation to finish:

<img src="https://s2.ax1x.com/2019/09/16/nWeaJP.png" width=500>

Click Next once the installation is complete:

<img src="https://s2.ax1x.com/2019/09/16/nWedRf.png" width=500>

You should see a window popping up, as shown below:

<img src="https://s2.ax1x.com/2019/09/16/nWeBQS.png" width=500>

If you would like to start with a clean system, follow the instruction on this Window. We will be using the GAAS Mirror to recover the TX2, so we will go ahead and close this window. Note not to select "Remove Downloaded Files" option.


### Step Two: Enter the Recovery Mode

1. Unplug the TX2 power cord (Do not skip this step)
2. Plug the TX2 power cord and press on the recovery button right away for 3 seconds
3. Press the reset button once
4. Connect the TX2 to a computer. Note that if you are using a virtual environment such as VMWare, you will need to choose to connect your hardware to the virtual machine.

5. Once the TX2 is connected to a computer, enter the following command:

```lsusb```

6. If you see the following output, it means you have successfully entered the recovery mode:

<img src="https://s2.ax1x.com/2019/09/16/nWeDsg.png" width=500>

### Step Three: Recover the TX2 with GAAS Mirror

1. Go to the Installation Directory for NVIDIA JetPack. You should see the folder 64_TX2, as shown below:

<img src="https://s2.ax1x.com/2019/09/16/nWewz8.png" width=500>

2. Go to Installation_Directory/64_TX2/Linux_for_tegra. Launch a terminal and copy tx2gaas.img to system.img:

```sudo cp my_backup.img system.img```
 
3. Go to the bootloader folder under JetPack Installation Directory to delete or backup the original system.img. We are going to back up system.img for this tutorial.

```
cd bootloader
mv system.img system_bak.img.bak
```
 
4. Then move tx2gaas mirror file (now it's named system.img) to the bootloader folder:
mv Linux_for_tegra/system.img system.img 
 
5. Go back to the directory JetPack Installation Directory and start to recover the TX2:

``` 
sudo ./flash.sh -r jetson-tx2 mmcblk0p1
```
 
The process takes 20-40 minutes. Once the recovery is done, restart the TX2 and start using GAAS on TX2.

### What to do next?
Now that you have the GAAS on your TX2 framework, you may want to try out some of our previous tutorials on a real drones:

[E03: Using SLAM In GPS Denied Environment For Position Estimation](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-part-3-using-slam-in-gps-denied-environment-for-position-estimation)

[E04: Depth Estimation, Octomap and Path Planning](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-part-4-stereo-depth-estimation-octomap-and-path-planning)

[E05: Vision-Based Auto-Landing](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-part-5-vision-based-auto-landing)

[E06: Basic Object Tracking](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/basic-object-tracking)
