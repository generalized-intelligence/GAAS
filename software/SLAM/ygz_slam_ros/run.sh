cd temp_img/keyframe_loopclosure
rm *.png

cd ../keyframe_neighbour
rm *.png

cd ../..
rm test.log
./bin/EurocStereoVIO_ros >> test.log
