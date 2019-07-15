README                        {#mainpage}
======

Welcome to OKVIS: Open Keyframe-based Visual-Inertial SLAM. 

This is the Author's implementation of the [1] and [3] with more results in [2].

[1] Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart and Paul 
    Timothy Furgale. Keyframe-based visual–inertial odometry using nonlinear 
    optimization. The International Journal of Robotics Research, 2015.

[2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for 
    Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

[3] Stefan Leutenegger, Paul Timothy Furgale, Vincent Rabaud, Margarita Chli, 
    Kurt Konolige, Roland Siegwart. Keyframe-Based Visual-Inertial SLAM using 
    Nonlinear Optimization. In Proceedings of Robotics: Science and Systems, 
    2013.

Note that the codebase that you are provided here is free of charge and without 
any warranty. This is bleeding edge research software.

Also note that the quaternion standard has been adapted to match Eigen/ROS, 
thus some related mathematical description in [1,2,3] will not match the 
implementation here.

If you publish work that relates to this software, please cite at least [1].

### License ###

The 3-clause BSD license (see file LICENSE) applies.

### How do I get set up? ###

This is a pure cmake project. 

You will need to install the following dependencies,

* CMake,

        sudo apt-get install cmake

* google-glog + gflags,

        sudo apt-get install libgoogle-glog-dev
   
* BLAS & LAPACK,

        sudo apt-get install libatlas-base-dev

* Eigen3,

        sudo apt-get install libeigen3-dev

* SuiteSparse and CXSparse,

        sudo apt-get install libsuitesparse-dev

* Boost,

        sudo apt-get install libboost-dev libboost-filesystem-dev

* OpenCV 2.4-3.0: follow the instructions on http://opencv.org/ or install 
  via 
 
        sudo apt-get install libopencv-dev

* Optional: use the the package with the Skybotix VI sensor.
  Note that this requires a system install, not just as ROS package. Also note 
  that Skybotix OSX support is experimental (checkout the feature/osx branch).

        git clone https://github.com/ethz-asl/libvisensor.git
        cd libvisensor
        ./install_libvisensor.sh

then download and expand the archive:

    wget https://www.doc.ic.ac.uk/~sleutene/software/okvis-1.1.3.zip
    unzip okvis-1.1.3.zip && rm okvis-1.1.3.zip

Or, if you were given bitbucket access, clone the repository:

    git clone git@github.com:ethz-asl/okvis.git

or

    git clone https://github.com/ethz-asl/okvis.git

### Building the project ###

To change the cmake build type for the whole project use:

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j8

NOTE: if you want to use the library, install the project (default or somewhere
else), so the dependencies can be resolved. 

    make install
    
### Running the demo application ###

You will find a demo application in okvis_apps. It can process datasets in the 
ASL/ETH format.

In order to run a minimal working example, follow the steps below:

1. Download a dataset of your choice from 
   http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. 
   Assuming you downloaded MH_01_easy/. 
   You will find a corresponding calibration / estimator configuration in the 
   config folder.

2. Run the app as

        ./okvis_app_synchronous path/to/okvis/config/config_fpga_p2_euroc.yaml path/to/MH_01_easy/mav0/

### Outputs and frames

In terms of coordinate frames and notation, 

* W denotes the OKVIS World frame (z up), 
* C\_i denotes the i-th camera frame 
* S denotes the IMU sensor frame
* B denotes a (user-specified) body frame.

The output of the okvis library is the pose T\_WS as a position r\_WS and quaternion 
q\_WS, followed by the velocity in World frame v\_W and gyro biases (b_g) as well as 
accelerometer biases (b_a). See the example application to get an idea on how to
use the estimator and its outputs (callbacks returning states).

### Configuration files ###

The config folder contains example configuration files. Please read the
documentation of the individual parameters in the yaml file carefully. 
You have various options to trade-off accuracy and computational expense as well 
as to enable online calibration.

### HEALTH WARNING: calibration ###

If you would like to run the software/library on your own hardware setup, be 
aware that good results (or results at all) may only be obtained with 
appropriate calibration of the 

* camera intrinsics,
* camera extrinsics (poses relative to the IMU), 
* knowledge about the IMU noise parameters,
* and ACCURATE TIME SYNCHRONISATION OF ALL SENSORS.

To perform a calibration yourself, we recommend the following:

* Get Kalibr by following the instructions here 
  https://github.com/ethz-asl/kalibr/wiki/installation . If you decide to build 
  from source and you run ROS indigo checkout pull request 3:

        git fetch origin pull/3/head:request3
        git checkout request3

* Follow https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration to 
  calibrate intrinsic and extrinsic parameters of the cameras. If you receive an 
  error message that the tool was unable to make an initial guess on focal 
  length, make sure that your recorded dataset contains frames that have the 
  whole calibration target in view.

* Follow https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration to get 
  estimates for the spatial parameters of the cameras with respect to the IMU.

### Using the library

Here's a minimal example of your CMakeLists.txt to build a project using
OKVIS.

    cmake_minimum_required(VERSION 2.8)
    
    set(OKVIS_INSTALLATION <path/to/install>) # point to installation
    
    # require OpenCV
    find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
    include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 
    
    # require okvis
    find_package( okvis 1.1 REQUIRED)
    include_directories(${OKVIS_INCLUDE_DIRS})
    
    # require brisk
    find_package( brisk 2 REQUIRED)
    include_directories(${BRISK_INCLUDE_DIRS})
    
    # require ceres
    list(APPEND CMAKE_PREFIX_PATH ${OKVIS_INSTALLATION})
    find_package( Ceres REQUIRED )
    include_directories(${CERES_INCLUDE_DIRS}) 
    
    # require OpenGV
    find_package(opengv REQUIRED)
    
    # VISensor, if available
    list(APPEND CMAKE_MODULE_PATH ${OKVIS_INSTALLATION}/lib/CMake)
    find_package(VISensor)
    if(VISENSORDRIVER_FOUND)
      message(STATUS "Found libvisensor.")
    else()
      message(STATUS "libvisensor not found")
    endif()
    
    # now continue with your project-specific stuff...

### Contribution guidelines ###

* Contact s.leutenegger@imperial.ac.uk to request access to the bitbucket 
  repository.

* Programming guidelines: please follow 
  https://github.com/ethz-asl/programming_guidelines/wiki/Cpp-Coding-Style-Guidelines .
	
* Writing tests: please write unit tests (gtest).

* Code review: please create a pull request for all changes proposed. The pull 
  request will be reviewed by an admin before merging.

### Support ###

The developpers will be happy to assist you or to consider bug reports / feature 
requests. But questions that can be answered reading this document will be 
ignored. Please contact s.leutenegger@imperial.ac.uk.
