## TODO
- 将现有的双目ORB+IMU跑起来(almost done, single thread)
- 后端的sliding window
- 当Point的Ref出窗时，应该换ref还是marg还是固定？
- 新关键帧策略，NeedNewKeyFrame()改成类似ORB
- 多线程
- 删掉冗余KF（未知bug。。。）
- PureVision下，在LocalBA之后，应该根据上一关键帧的位姿变化，对mpLastFrame的位姿进行更新。
- 带IMU的程序，对于静止和weak状态仍没有好好考虑。（静止状态，MH01用30KF，DeleteRedundant可以运行了。weak的还没考虑）
- check why backend estimated pose will jump in some special cases.
- DeleteRedundant如果放在LocalBA之前，会有seg fault。没找到为什么…… （Frame.mFeaturesLeft都没法调用.size()？）
- Debug the WEAK mode in IMU
- 如果关键帧过于重叠，后端优化的问题可能有些病态
- 不定期出现的Segment fault
- 更稳定的光流

## WEAK 模式的处理
- IMU 量重新初始化
- 优化时固定零偏
- WEAK 时后端如何操作? WEAK时后端带IMU优化容易飞走
- 检测视觉频繁丢失的情况 
- 从WEAK恢复的时候调整关键帧策略
- 暂时结论：
1. 整个系统reset效果不错

## NOTE 
- OpenCV的GFTT每调用一次开销约在10+ms，请谨慎使用
- Inv depth的后端优化总有一些莫明其妙的问题，位姿非常的不稳定？
- 前端优化的位姿在MH_05的前面100帧左右出现明显跳动


## DONE.
- 前端光流
- check "v is nullptr in Local ba without imu"(done.)
- check why backend estimated pose will jump in some special cases.(done.原因：后端有一部分地图点未更新世界坐标)
- check 为什么nRefMatches比Tracker中currentFrame的inliers还要少？(done. 正常)
- Consider add inv depth prior in estimation (done.)
 

## 日志
### 7.8
- 修正了带IMU优化的多线程后端

### 7.6
- 增加了多线程后端
- 将FAST+自带光流改回GFTT+cv光流。仅在关键帧处提新特征点
- 改正了IMMATURE判GOOD处的计算
- 修正了Viewer中锁过大的问题

### 6.27
- 重新整理了整个双目光流的过程，使之更加整洁
- 在后端增加了一个基于XYZ坐标的优化。似乎比逆深度参数化稳定一些？

### 6.21
- add temporal map point in non-keyframe 

### 6.19
- 在LK处增加了根据RANSAC拒绝部分追踪点的做法
- 在前端增加了一些临时地图点以增强稳定性

### 6.14
- adjust some thresholds in IMU related optimization

### 6.13
- 修改了IMU初始化策略
- 在test/testTrackerAll.cpp中测试整个流程

### 6.12 
- add clahe into frame creation to balance the grayscale histogram.
- add a grid in LK tracking and making features will not be too large.
- fix the bugs caused by not updating the world position of map points in backend optimization. The pose will not jump now. 
- 把带IMU的过程调通，修掉一些bug。仍存问题
- 调整DeleteRedundant位置，放在DeleteKF(0)之前。在window包括30KF的情况下，可以处理MH01的静止场景

### 6.10 
- add LK tracker, derived from ORB Tracker, see cv/src/TrackerLK.cpp 
- There are so many points ...

### 6.9 
- add good feature to track and opencv's optical flow

### 6.8 
- 增加了左右目间的光流计算视差的方法，但对近处效果不是很好。相比原版orb稍快一些。

### 6.7
- fixbug "v is nullptr in Local ba without imu"： CleanMapPoint时把expired的mpRefKF从MapPoint的mObservations中删掉
- 增加了光流部分的计算，见cv/Align.cpp, cv/LKFlow.cpp，测试见test/testLKFlow.cpp
- 光流参数见align.h中的常量，减小patch对速度提升明显。

### 6.6
- 把shared_ptr相关修改合并到master
- EdgeProjectPoseOnly的computeError()里，如果invz<0，不能把_error置零，否则会当成在poseoptimization中会当成inlier
- 测试纯视觉的方案，见test/testPureVision.cpp
- wj-增加NeedNewKeyFrame()逻辑
- wj-Tracker的OptimizePose中，不加入Observations个数小于1的点（TBD）

### 6.5
- 把很多内存相关的东西都改成了shared_ptr和weak_ptr，再也不用担心内存泄漏啦！

### 6.4
- wj:
- Frame::SetDelete在修改完shared_ptr之后，要根据情况再考虑

### 6.3
- xiang:
- 16.04或同版本linux下，g2o/core/jacobian_workspace.cpp里有一句setZero，需要加上维度。否则易导致在开辟雅可比空间时seg fault.

### 6.2
- fix many Eigen::aligned_allocator problem
- rewrite the test program in test/xxx
- add stereo imu initialization code
- wj:
- 在imuinitialization中增加g=9.81的约束，加一步估计步骤。
- 在后端纯视觉的LocalBA中加入：对outlier的观测进行删除
- imuinitialization的成功条件：无约束估计出的g的模长在9.6~10.0之间，并且约束g=9.81前后估计出加速度计零偏的误差模长小于0.2m/s^2
  注：看log拍脑袋想的条件。。TBD
- 在V101中，31个KF左右可以满足条件（从第6s开始。前6s没动，测试时被跳过），MH01要50多个KF。估计出的零偏可能还有误差，等后续继续优化。
- imuinitialization成功后，将w系与重力方向对齐，使mgWrold=[0,0,9.81]，这样可以认为这个重力是真值，不需要对它进行优化更新。


### 5.31
- 修改了testViewer里的内存管理问题。 

### 5.30 
- 增加了后端新增地图点的过程，但未测试

### 5.29
- fix many things in tracker and ORBExtractor, ORBMatcher by testing stereo init. 


### 5.27
- 增加了Viewer的测试，见test/testViewer.cpp
- 由于pangolin/OpenGL的问题，在osx下新开openGL线程时会导致出错，所以testViewer在主线程中调用可视化

### 5.21
- 准备开始测试带IMU的双目初始化部分代码 

### 5.20 
- 增加了带IMU的Local BA测试，见test/testLocalBAIMU
- 在osx下，g2o的optimizer在析构，删除顶点时会产生double free问题，原因不明。Ubuntu下没有问题。

### 5.17
- 后端LocalBA通过测试，见test/testLocalBA

### 5.12
- 添加了后端的两个BA，待测试

### 5.10
- 添加一些g2otypes，使用P+R,V,Ba,Bg进行基本的表示 
- 开始添加Tracker内容

### 4.27 数据结构基本完成，加入特征提取（待测试）
### 2017.4.26 调整架构，加入前端算法
