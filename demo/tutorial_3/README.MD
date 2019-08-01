## Using SLAM In GPS Denied Environment For Position Estimation
In this tutorial we will talk about how to provide accurate position estimation from SLAM (Simultaneous localization and mapping) and control your drone in GPS denied environment.

Step-by-step instruction: https://gaas.gitbook.io/guide/

This tutorial has been validated on a real drone but the performance is not stable and there is plenty of room for improvement. Therefore, I would recommend you to proceed with extreme caution and do not test it on a real drone before you are ready. In this tutorial we will test it in Gazebo and the setup procedure is pretty much the same as a real drone.
This tutorial can be organized into the following part:
1. Environment update and setup;
2. Building SLAM and its usage;
3. Test it in Gazebo.

---
## 在无GPS环境下通过SLAM实现位置估计
本课中我们将会讲解如何通过SLAM（Simultaneous localization and mapping， 即实时定位与建图）实现无GPS环境下无人机的位置估计。

我们已经推出了开发者套件，并且可以预装 GAAS 以及 Mavros。你可以自己搭配，也可以选择我们为你[组合好的开发平台](https://item.taobao.com/item.htm?id=591140560551).

手把手教学：https://gaas.gitbook.io/guide/

本讲内容在实机上通过测试，可以实现较为稳定的定点，但是还有很大的优化空间。大家没有把握的情况下我不建议实机测试，因为当SLAM信号中断时会有潜在危险。本讲中的内容会通过Gazebo展示，但是实机上的设置方法也非常类似。
本讲内容可以分为三个部分：
1. 环境更新及设置；
2. SLAM及使用；
3. Gazebo验证。
