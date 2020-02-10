**一个简单的多目SLAM实现 支持多组双目或多组深度相机**

依赖项:
1.opencv 3.4.5
2.cv_bridge 下载后在catkin_ws/src中使用catkin build编译(generate.sh运行前需要source 对应的devel/setup.bash)
3.gtsam 4.0
4.glog
5.PCL


TODO LIST:

1.关键帧策略加入追踪点数量关系.
2.加入backend thread.
3.前端多线程化.



