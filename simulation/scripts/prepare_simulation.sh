export PX4_ENV_PATH="/home/gi/Downloads/PX4_ENV_bak" # path to your px4 1.8
export SF="/src/Firmware"
export LAUNCH_FILE="mavros_posix_sitl.launch"

cd $PX4_ENV_PATH
catkin build
cd $PX4_ENV_PATH$SF
source ./Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 $LAUNCH_FILE
