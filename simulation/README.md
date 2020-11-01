这里存放模拟器环境需要的机型模型和环境地图.
可以按需组合.

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




Start simulation:Edit 'script/prepare_simulation.sh' with your own px4 path and run this script.
Copy simulation_with_velodyne_in_gazebo9_ws/src/ to your own px4 workspace and run 'catkin build', to compile lidar module.

Do:

cp -r ${YOUR_GAAS_PATH}/simulator/models/* ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/models/

cp -r ${YOUR_GAAS_PATH}/simulator/worlds/* ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/worlds/

cp -r ${YOUR_GAAS_PATH}/simulator/posix-config/* ${YOUR_PX4_WORKSPACE}/src/Firmware/posix-configs/SITL/init/ekf2/


cp -r ./simulation_with_velodyne_in_gazebo9_ws/src/velodyne_simulator/velodyne_description/  ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/models/
cp -r ./simulation_with_velodyne_in_gazebo9_ws/src/velodyne_simulator  ${YOUR_PX4_WORKSPACE}/src

cp -r ./models/*  ${YOUR_PX4_WORKSPACE}/src/Firmware/Tools/sitl_gazebo/models/  
# To get this sdf, run xacro src/velodyne_simulator/velodyne_description/urdf/example.urdf.xacro > example.urdf"
# and "gz sdf -p example.urdf >example.sdf"
# Then manually edit your drone's sdf file and add the elements you need for simulation from example.sdf.


cp -r ./ekf_settings/* ${YOUR_PX4_WORKSPACE}/src/Firmware/posix-configs/SITL/init/ekf2/



and change the launch file params in 'script/prepare_simulation.sh' to utilize drones with stereo cam or lidar.


