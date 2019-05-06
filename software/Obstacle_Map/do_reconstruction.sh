echo "RUN DENSE_RECONSTRUCTION"
#./bin/dense_reconstruction  -l /mynteye/right/image_rect -r /mynteye/left/image_rect -c calibration/mynteye_sim2.yml -w 752 -h 480 -u 752 -v 480
./bin/dense_reconstruction  -l /gi/simulation/left/image_raw -r /gi/simulation/right/image_raw -c calibration/mynteye_sim2.yml -w 752 -h 480 -u 752 -v 480
