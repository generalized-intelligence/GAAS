echo "RUN DENSE_RECONSTRUCTION"
#./bin/dense_reconstruction  -l /mynteye/left/image_rect -r /mynteye/right/image_rect -c calibration/mynteye.yml -w 752 -h 480 -u 376 -v 240
#./bin/dense_reconstruction  -l /multisense_sl/camera/right/image_raw -r /multisense_sl/camera/left/image_raw -c calibration/mynteye_sim2.yml -w 752 -h 480 -u 752 -v 480
./bin/dense_reconstruction  -l /multisense_sl/camera/right/image_raw -r /multisense_sl/camera/left/image_raw -c calibration/mynteye_sim2.yml -w 752 -h 480 -u 376 -v 240
