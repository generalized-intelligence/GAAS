使用方法：
在当前目录直接执行: catkin_make_isolated
如果不能构建成功，尝试删除devel_isolated 和 build_isolated,并重新执行一次catkin_make_isolated.第一次执行会提示错误，找到build_isolated/okvis_ros/okvis/ceres/src/ceres_external/CMakeLists.txt中的 '-Werror' 删除它并再次执行catkin_make_isolated.
建议使用kalibr进行相机-IMU联合标定。
