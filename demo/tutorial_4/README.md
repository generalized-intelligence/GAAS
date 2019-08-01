<img width="300px" src="https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-LYUhlGdK9Y1iLhupMFC%2F-Le-pbmWyVALS-76fVNx%2F-Le-pgaHG2WlZufxbZO3%2Ft4-A*-1.png?alt=media&token=cd0b31ea-645d-48e1-830b-535001f1fae8"/>


## Depth Estimation, Octomap and Path Planning
In the previous tutorials, we have talked about how to control your drone with python and enable your drone to fly in GPS denied environments, which is a big step towards autonomous drone. Now, we will talk about how to enable your drone to fly in an unknown environment from point A to point B, with the help of a pair of stereo camera only, in Gazebo simulator. This tutorial can be organized as follow:
1. Obstacle distance estimation with Stereo Camera;
2. Octomap as a way to represent the environment;
3. A* path finding in 3D space;
4. Simple path pruning;


Again, I have to stress that you will need some extra work before going to the field test and the algorithms mentioned here are far from optimal. 

To ensure optimal reading experience, [continue reading](https://gaas.gitbook.io/guide/) this tutorial at our dedicated GitBook.

---

## 深度估计，八叉树地图以及路径规划
前面的几讲中，我们讨论了如何通过python控制无人机以及如何控制无人机在没有GPS的环境下飞行，这些内容是通往无人机自动驾驶的重要一步。本讲中，我们将讨论如何控制无人机在一个未知环境下，只通过一组双目摄像头，在Gazebo中实现路径规划，从点A飞行到点B。本讲内容可以分为如下几个部分：
1. 基于双目摄像头的景深估计；
2. 使用Octomap表示环境；
3. 3D空间中的A*路径规划；
4. 在Gazebo中实验飞行；
5. 路径修剪。


我需要强调的是，我们提供的算法并不是最优的，你可能需要进行参数调优，甚至更换其中的某一或某几个部分的算法，才能实现较为稳定的实际飞行效果。

为了保证最佳阅读体验，我们将本次教程放到了 GitBook 文档中，[点击这里](https://gaas.gitbook.io/guide/)继续阅读。
