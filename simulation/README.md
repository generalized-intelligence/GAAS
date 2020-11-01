这里存放模拟器环境需要的机型模型和环境地图.

可以按需组合.

由于PX4 1.8.0直接用catkin build编译会有问题,需要补丁.请做如下手动更改.

Download PX4 Firmware and checkout to branch "v1.8.0".

At Firmware/CMakeLists.txt, patch:

@@ -185,6 +185,11 @@ set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
 set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PX4_BINARY_DIR})
 set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX4_BINARY_DIR})
 set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX4_BINARY_DIR})
+if (CATKIN_DEVEL_PREFIX)
+       SET(BUILD_SHARED_LIBS OFF)
+endif()

To avoid ecl compilation error.

来避免ecl编译错误.


要开始仿真:编辑'script/prepare_simulation.sh', 填入你自己的px4工程路径,并运行这个脚本.

Start simulation:Edit 'script/prepare_simulation.sh' with your own px4 path and run this script.

Copy simulation_with_velodyne_in_gazebo9_ws/src/ to your own px4 workspace and run 'catkin build', to compile lidar module.

Do:

cp -r ${YOUR_GAAS_PATH}/simulator/models/* ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/models/

cp -r ${YOUR_GAAS_PATH}/simulator/worlds/* ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/worlds/

cp -r ${YOUR_GAAS_PATH}/simulator/posix-config/* ${YOUR_PX4_WORKSPACE}/src/Firmware/posix-configs/SITL/init/ekf2/

\#这三步与GAAS相同.

cp -r ./simulation_with_velodyne_in_gazebo9_ws/src/velodyne_simulator/velodyne_description/  ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/models/

cp -r ./simulation_with_velodyne_in_gazebo9_ws/src/velodyne_simulator  ${YOUR_PX4_WORKSPACE}/src

cp -r ./models/*  ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/models/  

\# 如果你好奇生成sdf文件的步骤:

\# To get this sdf, run "xacro src/velodyne_simulator/velodyne_description/urdf/example.urdf.xacro > example.urdf"

\# and "gz sdf -p example.urdf >example.sdf"

\# Then manually edit your drone's sdf file and add the elements you need for simulation from example.sdf.


cp -r ./ekf_settings/* ${YOUR_PX4_WORKSPACE}/src/Firmware/posix-configs/SITL/init/ekf2/

and change the launch file params in 'script/prepare_simulation.sh' to utilize drones with stereo cam or lidar.

运行'script/prepare_simulation.sh', 启动激光雷达与双目视觉在旋翼机上仿真.


