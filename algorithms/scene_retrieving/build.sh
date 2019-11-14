cd ..
cd loop_closing/DBow3
sh build.sh
cd ../../scene_retrieving

mkdir build
cd build && cmake ..
make -j8
cd ..
