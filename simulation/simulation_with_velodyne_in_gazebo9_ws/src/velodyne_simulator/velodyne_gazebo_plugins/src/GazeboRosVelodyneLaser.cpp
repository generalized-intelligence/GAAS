/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <velodyne_gazebo_plugins/GazeboRosVelodyneLaser.h>

#include <algorithm>
#include <assert.h>
#include <memory>

#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
//#include <Pose.hh>
using namespace ignition;
namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVelodyneLaser::GazeboRosVelodyneLaser() : rosnode_(NULL), laser_connect_count_(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  laser_queue_.clear();
  laser_queue_.disable();
  if (rosnode_) {
    rosnode_->shutdown();
    delete rosnode_;
    rosnode_ = NULL;
  }
  callback_laser_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  parent_sensor_ = _parent;

  // Get the world name.
  std::string worldName = _parent->WorldName();
  world_ = physics::get_world(worldName);

  last_update_time_ = world_->SimTime();

  node_ = transport::NodePtr(new transport::Node());
  node_->Init(worldName);

  parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(parent_sensor_);

  if (!parent_ray_sensor_) {
    gzthrow("GazeboRosVelodyneLaser controller requires a Ray Sensor as its parent");
  }

  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (!_sdf->HasElement("frameName")) {
    ROS_INFO("Velodyne laser plugin missing <frameName>, defaults to /world");
    frame_name_ = "/world";
  } else {
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("min_range")) {
    ROS_INFO("Velodyne laser plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  } else {
    min_range_ = _sdf->GetElement("min_range")->Get<double>();
  }

  if (!_sdf->HasElement("max_range")) {
    ROS_INFO("Velodyne laser plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  } else {
    max_range_ = _sdf->GetElement("max_range")->Get<double>();
  }

  if (!_sdf->HasElement("topicName")) {
    ROS_INFO("Velodyne laser plugin missing <topicName>, defaults to /world");
    topic_name_ = "/world";
  } else {
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("gaussianNoise")) {
    ROS_INFO("Velodyne laser plugin missing <gaussianNoise>, defaults to 0.0");
    gaussian_noise_ = 0;
  } else {
    gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  rosnode_->getParam(std::string("tf_prefix"), prefix);
  frame_name_ = tf::resolve(prefix, frame_name_);

  if (topic_name_ != "") {
    // Custom Callback Queue
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
      topic_name_,1,
      boost::bind( &GazeboRosVelodyneLaser::laserConnect,this),
      boost::bind( &GazeboRosVelodyneLaser::laserDisconnect,this), ros::VoidPtr(), &laser_queue_);
    pub_ = rosnode_->advertise(ao);
  }

  // sensor generation off by default
  parent_ray_sensor_->SetActive(false);
  // start custom queue for laser
  callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosVelodyneLaser::laserQueueThread,this ) );

  ROS_INFO("Velodyne laser plugin ready, %i lasers", parent_ray_sensor_->VerticalRangeCount());
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosVelodyneLaser::laserConnect()
{
  laser_connect_count_++;
  parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosVelodyneLaser::laserDisconnect()
{
  laser_connect_count_--;
  if (laser_connect_count_ == 0) {
    parent_ray_sensor_->SetActive(false);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVelodyneLaser::OnNewLaserScans()
{
  if (topic_name_ != "") {
    common::Time sensor_update_time = parent_sensor_->LastUpdateTime();
    if (last_update_time_ < sensor_update_time) {
      putLaserData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  } else {
    ROS_INFO("gazebo_ros_velodyne_laser topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosVelodyneLaser::putLaserData(common::Time &_updateTime)
{
  int i, hja, hjb;
  int j, vja, vjb;
  double vb, hb;
  int    j1, j2, j3, j4; // four corners indices
  double r1, r2, r3, r4, r; // four corner values + interpolated range
  double intensity;

  parent_ray_sensor_->SetActive(false);

  math::Angle maxAngle = parent_ray_sensor_->AngleMax();
  math::Angle minAngle = parent_ray_sensor_->AngleMin();

  double maxRange = parent_ray_sensor_->RangeMax();
  double minRange = parent_ray_sensor_->RangeMin();
  int rayCount = parent_ray_sensor_->RayCount();
  int rangeCount = parent_ray_sensor_->RangeCount();

  int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
  int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();
  math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
  math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();

  double yDiff = maxAngle.Radian() - minAngle.Radian();
  double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  boost::mutex::scoped_lock lock(lock_);

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp.sec = _updateTime.sec;
  msg.header.stamp.nsec = _updateTime.nsec;
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange - minRange - 0.01);

  uint8_t *ptr = msg.data.data();
  for (j = 0; j<verticalRangeCount; j++) {
    // interpolating in vertical direction
    vb = (verticalRangeCount == 1) ? 0 : (double) j * (verticalRayCount - 1) / (verticalRangeCount - 1);
    vja = (int) floor(vb);
    vjb = std::min(vja + 1, verticalRayCount - 1);
    vb = vb - floor(vb); // fraction from min

    assert(vja >= 0 && vja < verticalRayCount);
    assert(vjb >= 0 && vjb < verticalRayCount);

    for (i = 0; i<rangeCount; i++) {
      // Interpolate the range readings from the rays in horizontal direction
      hb = (rangeCount == 1)? 0 : (double) i * (rayCount - 1) / (rangeCount - 1);
      hja = (int) floor(hb);
      hjb = std::min(hja + 1, rayCount - 1);
      hb = hb - floor(hb); // fraction from min

      assert(hja >= 0 && hja < rayCount);
      assert(hjb >= 0 && hjb < rayCount);

      // indices of 4 corners
      j1 = hja + vja * rayCount;
      j2 = hjb + vja * rayCount;
      j3 = hja + vjb * rayCount;
      j4 = hjb + vjb * rayCount;
      // range readings of 4 corners
      r1 = std::min(parent_ray_sensor_->LaserShape()->GetRange(j1) , maxRange-minRange);
      r2 = std::min(parent_ray_sensor_->LaserShape()->GetRange(j2) , maxRange-minRange);
      r3 = std::min(parent_ray_sensor_->LaserShape()->GetRange(j3) , maxRange-minRange);
      r4 = std::min(parent_ray_sensor_->LaserShape()->GetRange(j4) , maxRange-minRange);

      // Range is linear interpolation if values are close,
      // and min if they are very different
      r = (1 - vb) * ((1 - hb) * r1 + hb * r2)
         +     vb  * ((1 - hb) * r3 + hb * r4);
      if (gaussian_noise_ != 0.0) {
        r += gaussianKernel(0,gaussian_noise_);
      }

      // Intensity is averaged
      intensity = 0.25*(parent_ray_sensor_->LaserShape()->GetRetro(j1) +
                        parent_ray_sensor_->LaserShape()->GetRetro(j2) +
                        parent_ray_sensor_->LaserShape()->GetRetro(j3) +
                        parent_ray_sensor_->LaserShape()->GetRetro(j4));

      // get angles of ray to get xyz for point
      double yAngle = 0.5*(hja+hjb) * yDiff / (rayCount -1) + minAngle.Radian();
      double pAngle = 0.5*(vja+vjb) * pDiff / (verticalRayCount -1) + verticalMinAngle.Radian();

      //pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle);
        *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle);
#if GAZEBO_MAJOR_VERSION > 2
        *((float*)(ptr + 8)) = r * sin(pAngle);
#else
        *((float*)(ptr + 8)) = -r * sin(pAngle);
#endif
        *((float*)(ptr + 16)) = intensity;
#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t*)(ptr + 20)) = j; // ring
#else
        *((uint16_t*)(ptr + 20)) = verticalRangeCount - 1 - j; // ring
#endif
        ptr += POINT_STEP;
      }
    }
  }
  parent_ray_sensor_->SetActive(true);

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step); // Shrink to actual size

  // Publish output
  pub_.publish(msg);
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosVelodyneLaser::laserQueueThread()
{
  static const double TIMEOUT = 0.01;

  while (rosnode_->ok()) {
    laser_queue_.callAvailable(ros::WallDuration(TIMEOUT));
  }
}

void GazeboRosVelodyneLaser::onStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  sim_time_  = msgs::Convert( _msg->sim_time() );

  //math::Pose pose;
  math::Pose3d pose;
  //pose.pos.x = 0.5*sin(0.01*sim_time_.Double());
  pose.Pos()[0] = 0.5*sin(0.01*sim_time_.Double());
  gzdbg << "plugin simTime [" << sim_time_.Double() << "] update pose [" << pose.Pos()[0] << "]\n";
}

}
