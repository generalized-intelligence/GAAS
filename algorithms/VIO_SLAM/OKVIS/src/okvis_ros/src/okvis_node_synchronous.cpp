/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jun 26, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis_node_synchronous.cpp
 * @brief This file includes the synchronous ROS node implementation.
 
          This node goes through a rosbag in order and waits until all processing is done
          before adding a new message to algorithm

 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>

#include "sensor_msgs/Imu.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <okvis/Subscriber.hpp>
#include <okvis/Publisher.hpp>
#include <okvis/RosParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"


// this is just a workbench. most of the stuff here will go into the Frontend class.
int main(int argc, char **argv) {

  ros::init(argc, argv, "okvis_node_synchronous");

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  if (argc != 3 && argc != 4) {
    LOG(ERROR) <<
        "Usage: ./" << argv[0] << " configuration-yaml-file bag-to-read-from [skip-first-seconds]";
    return -1;
  }

  okvis::Duration deltaT(0.0);
  if (argc == 4) {
    deltaT = okvis::Duration(atof(argv[3]));
  }

  // set up the node
  ros::NodeHandle nh("okvis_node");

  // publisher
  okvis::Publisher publisher(nh);

  // read configuration file
  std::string configFilename(argv[1]);

  okvis::RosParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);

  okvis::ThreadedKFVio okvis_estimator(parameters);

  okvis_estimator.setFullStateCallback(std::bind(&okvis::Publisher::publishFullStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
  okvis_estimator.setLandmarksCallback(std::bind(&okvis::Publisher::publishLandmarksAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
  okvis_estimator.setStateCallback(std::bind(&okvis::Publisher::publishStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2));
  okvis_estimator.setBlocking(true);
  publisher.setParameters(parameters); // pass the specified publishing stuff

  // extract the folder path
  std::string bagname(argv[2]);
  size_t pos = bagname.find_last_of("/");
  std::string path;
  if (pos == std::string::npos)
    path = ".";
  else
    path = bagname.substr(0, pos);

  const unsigned int numCameras = parameters.nCameraSystem.numCameras();

  // setup files to be written
  publisher.setCsvFile(path + "/okvis_estimator_output.csv");
  publisher.setLandmarksCsvFile(path + "/okvis_estimator_landmarks.csv");
  okvis_estimator.setImuCsvFile(path + "/imu0_data.csv");
  for (size_t i = 0; i < numCameras; ++i) {
    std::stringstream num;
    num << i;
    okvis_estimator.setTracksCsvFile(i, path + "/cam" + num.str() + "_tracks.csv");
  }

  // open the bag
  rosbag::Bag bag(argv[2], rosbag::bagmode::Read);
  // views on topics. the slash is needs to be correct, it's ridiculous...
  std::string imu_topic("/imu0");
  rosbag::View view_imu(
      bag,
      rosbag::TopicQuery(imu_topic));
  if (view_imu.size() == 0) {
    LOG(ERROR) << "no imu topic";
    return -1;
  }
  rosbag::View::iterator view_imu_iterator = view_imu.begin();
  LOG(INFO) << "No. IMU messages: " << view_imu.size();

  std::vector<std::shared_ptr<rosbag::View> > view_cams_ptr;
  std::vector<rosbag::View::iterator> view_cam_iterators;
  std::vector<okvis::Time> times;
  okvis::Time latest(0);
  for(size_t i=0; i<numCameras;++i) {
    std::string camera_topic("/cam"+std::to_string(i)+"/image_raw");
    std::shared_ptr<rosbag::View> view_ptr(
        new rosbag::View(
            bag,
            rosbag::TopicQuery(camera_topic)));
    if (view_ptr->size() == 0) {
      LOG(ERROR) << "no camera topic";
      return 1;
    }
    view_cams_ptr.push_back(view_ptr);
    view_cam_iterators.push_back(view_ptr->begin());
    sensor_msgs::ImageConstPtr msg1 = view_cam_iterators[i]
        ->instantiate<sensor_msgs::Image>();
    times.push_back(
        okvis::Time(msg1->header.stamp.sec, msg1->header.stamp.nsec));
    if (times.back() > latest)
      latest = times.back();
    LOG(INFO) << "No. cam " << i << " messages: " << view_cams_ptr.back()->size();
  }

  for(size_t i=0; i<numCameras;++i) {
    if ((latest - times[i]).toSec() > 0.01)
      view_cam_iterators[i]++;
  }

  int counter = 0;
  okvis::Time start(0.0);
  while (ros::ok()) {
    ros::spinOnce();
	okvis_estimator.display();

    // check if at the end
    if (view_imu_iterator == view_imu.end()){
      std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
      char k = 0;
      while(k==0 && ros::ok()){
        k = cv::waitKey(1);
        ros::spinOnce();
      }
      return 0;
    }
    for (size_t i = 0; i < numCameras; ++i) {
      if (view_cam_iterators[i] == view_cams_ptr[i]->end()) {
        std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
        char k = 0;
        while(k==0 && ros::ok()){
          k = cv::waitKey(1);
          ros::spinOnce();
        }
        return 0;
      }
    }

    // add images
    okvis::Time t;
    for(size_t i=0; i<numCameras;++i) {
      sensor_msgs::ImageConstPtr msg1 = view_cam_iterators[i]
          ->instantiate<sensor_msgs::Image>();
      cv::Mat filtered(msg1->height, msg1->width, CV_8UC1);
      memcpy(filtered.data, &msg1->data[0], msg1->height * msg1->width);
      t = okvis::Time(msg1->header.stamp.sec, msg1->header.stamp.nsec);
      if (start == okvis::Time(0.0)) {
        start = t;
      }

      // get all IMU measurements till then
      okvis::Time t_imu=start;
      do {
        sensor_msgs::ImuConstPtr msg = view_imu_iterator
            ->instantiate<sensor_msgs::Imu>();
        Eigen::Vector3d gyr(msg->angular_velocity.x, msg->angular_velocity.y,
                            msg->angular_velocity.z);
        Eigen::Vector3d acc(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);

        t_imu = okvis::Time(msg->header.stamp.sec, msg->header.stamp.nsec);

        // add the IMU measurement for (blocking) processing
        if (t_imu - start > deltaT)
          okvis_estimator.addImuMeasurement(t_imu, acc, gyr);

        view_imu_iterator++;
      } while (view_imu_iterator != view_imu.end() && t_imu <= t);

      // add the image to the frontend for (blocking) processing
      if (t - start > deltaT)
        okvis_estimator.addImage(t, i, filtered);

      view_cam_iterators[i]++;
    }
    ++counter;

    // display progress
    if (counter % 20 == 0) {
      std::cout
          << "\rProgress: "
          << int(double(counter) / double(view_cams_ptr.back()->size()) * 100)
          << "%  " ;
    }

  }

  std::cout << std::endl;
  return 0;
}
