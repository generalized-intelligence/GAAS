# A simple and naive BVT :)

cd algorithms&&catkin_make gaas_msgs_gencpp
cd build&&cmake ../src&&make gaas_msgs_gencpp&&make -j8 && cd ../../simulation&&catkin_make
