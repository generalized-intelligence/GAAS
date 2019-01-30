# Dense 3D Reconstruction from Stereo

This is a ROS package for real-time 3D reconstruction from stereo images. Currently this version uses [LIBELAS](http://www.cvlibs.net/software/libelas/) 
for generating dense disparity maps as a baseline. The method for generation of disparity maps can be changed based on user preferences.

This package serves as a visualization tool for dense disparity maps and point clouds. Additionally, a tool for transforming point clouds to a different 
reference frame is also included. 

Usually, the point clouds are formed in the reference frame of the left camera. For ground robots, often the point clouds need to be transformed to 
a different frame *e.g.*, a reference frame with the origin at the centre of rotation of the robot projected on to the ground plane. These transformations are 
hard to calculate mathematically - this tool can be used to find the transformations visually.

- Author: [Sourish Ghosh](http://sourishghosh.com/)

## Dependencies

- A C++ compiler (*e.g.*, [GCC](http://gcc.gnu.org/))
- [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
- [cmake](http://www.cmake.org/cmake/resources/software.html)
- [popt](http://freecode.com/projects/popt)
- [Boost](http://www.boost.org/)
- [OpenCV](https://github.com/opencv/opencv)

## Stereo Calibration

A calibrated pair of cameras is required for stereo rectification and calibration files should be stored in a `.yml` file. 
[This repository](https://github.com/sourishg/stereo-calibration) contains all the tools and instructions to calibrate stereo cameras.

The rotation and translation matrices for the point cloud transformation should be named as `XR` and `XT` in the calibration file. `XR` should be a **3 x 3** 
matrix and `XT` should be a **3 x 1** matrix. Please see a sample calibration file in the `calibration/` folder.

## Compiling

Clone the repository:

```bash
$ git clone https://github.com/umass-amrl/stereo_dense_reconstruction
```

For compiling the ROS package, `rosbuild` is used. Add the path of the ROS package to `ROS_PACKAGE_PATH` and put the following line in your `.bashrc` file. 
Replace `PATH` by the actual path where you have cloned the repository:

```bash
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PATH
```

Execute the `build.sh` script:

```bash
$ cd stereo_dense_reconstruction
$ chmod +x build.sh
$ ./build.sh
```

## Running Dense 3D Reconstruction

```bash
$ ./bin/dense_reconstruction [OPTION...]
```

```bash
Usage: dense_reconstruction [OPTION...]
  -l, --left_topic=STR       Left image topic name
  -r, --right_topic=STR      Right image topic name
  -c, --calib_file=STR       Stereo calibration file name
  -w, --calib_width=NUM      Calibration image width
  -h, --calib_height=NUM     Calibration image height
  -u, --out_width=NUM        Rectified image width
  -v, --out_height=NUM       Rectified image height
  -d, --debug=NUM            Set d=1 for cam to robot frame calibration
```

This node outputs the dense disparity map as a grayscale image on the topic `/camera/left/disparity_map` and the corresponding point cloud on the topic 
`/camera/left/point_cloud`.

A sample dataset can be found [here](https://greyhound.cs.umass.edu/owncloud/index.php/s/3g9AwCSkGi6LznK).

## Point Cloud Transformation

The point cloud can be viewed on `rviz` by running:

```bash
$ rosrun rviz rviz
```

To transform the point cloud to a different reference frame, the `XR` and `XT` matrices (rotation and translation) in the calibration file need to be changed. 
This can be done real-time by the running:

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

If you change the Euler Angles in `rqt_reconfigure` you should be able to see the point cloud transform. Don't forget to set `d=1` when running the 
`dense_reconstruction` node. This prints out the new transformation matrices as you transform the point cloud.

## License

This software is released under the [GNU GPL v3 license](LICENSE).