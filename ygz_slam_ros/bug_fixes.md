1. when pushing features points to vector<cv::Point3d>, there are cases when certain features can be zero, before pushing them to vector<cv::Point3d> one should remove them.

2. I completely commented out the localxyz optimization part which can be triggered during IMU initialization because it could lead to segmentation fault during "addEdge()"

3. Now Mynteyeofficial does not need to be added manualy as a parameter, directly run ./bin/EurocstereoVio_ros would be enough.

4. temp images for loop closure is saved in temp_img folder located in the project root.


