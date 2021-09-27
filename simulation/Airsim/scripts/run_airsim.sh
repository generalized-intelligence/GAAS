#tmux new -s init -d  # session name: init
#tmux rename-window -t "init:0" service  # window name: service

#tmux split-window -t "init:service"
#tmux send -t "init:service" "roscore" Enter


#tmux split-window -t "init:service"
#tmux send -t "init:0" "(cd ~/Downloads/Airsim/MSBuild2018/LinuxNoEditor && ./MSBuild2018.sh )" Enter


#tmux split-window -t "init:service"
#tmux send -t "init:service" "(cd ~/Downloads/Airsim/AirSim/PX4/PX4-Autopilot && make px4_sitl_default none_iris)" Enter


#tmux split-window -t "init:service"
#tmux send -h -t "init:service" "(roslaunch mavros px4_airsim.launch)" Enter

#tmux split-window -t "init:service"
#tmux send -h -t "init:service" "(cd ~/Downloads/Airsim/AirSim/ros && source devel/setup.bash && roslaunch airsim_ros_pkgs airsim_node.launch)" Enter


# session name: init
#sudo nvidia-smi --power-limit=90
tmux new -s init -d
tmux neww -n processes "(roscore)"
tmux neww -n processes "(cd ~/Downloads/Airsim/MSBuild2018/LinuxNoEditor && ./MSBuild2018.sh );sleep 10000"
tmux neww -n processes "(cd ~/Downloads/Airsim/AirSim/PX4/PX4-Autopilot && make px4_sitl_default none_iris)"
tmux neww -n processes "(sleep 5 && roslaunch mavros px4_airsim.launch;sleep 10000)"
tmux neww -n processes "(sleep 5 && cd ~/Downloads/Airsim/AirSim/ros && source devel/setup.bash && roslaunch airsim_ros_pkgs airsim_node.launch)"

tmux neww -n processes "sleep 8 && (cd ~/Downloads && ./QGroundControl.AppImage)"

#(cd ~/Downloads/Airsim/MSBuild2018/LinuxNoEditor && ./MSBuild2018.sh ) &
#(cd ~/Downloads/Airsim/AirSim/PX4/PX4-Autopilot && make px4_sitl_default none_iris ) &
#(roslaunch mavros px4_airsim.launch ) &
#(cd ~/Downloads/Airsim/AirSim/ros && source devel/setup.bash && roslaunch airsim_ros_pkgs airsim_node.launch) &
