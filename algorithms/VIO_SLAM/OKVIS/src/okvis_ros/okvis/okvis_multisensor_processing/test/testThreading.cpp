#define GTEST_USE_OWN_TR1_TUPLE 0

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/highgui/highgui.hpp>
#pragma GCC diagnostic pop

#include <okvis/kinematics/Transformation.hpp>

#include <okvis/ThreadedKFVio.hpp>

#include "MockVioBackendInterface.hpp"
#include "MockVioFrontendInterface.hpp"
#include "testDataGenerators.hpp"

using ::testing::AtLeast;
using ::testing::Between;
using ::testing::Return;
using ::testing::_;

TEST(OkvisVioInterfaces, testConstructionDestruction)
{
    using namespace okvis;

    MockVioBackendInterface dummy;
    EXPECT_CALL(dummy, addCamera(_))
      .Times(2);
    EXPECT_CALL(dummy, addImu(_))
      .Times(1);
    EXPECT_CALL(dummy, currentFrameId())
      .Times(1);
    EXPECT_CALL(dummy, get_T_WS(_,_))
      .Times(1);

    MockVioFrontendInterface mock_frontend;
    okvis::VioParameters parameters;
    parameters.nCameraSystem = TestDataGenerator::getTestCameraSystem(2);
    // start with mock

    EXPECT_CALL(mock_frontend, setBriskDetectionOctaves(_))
        .Times(1);
    EXPECT_CALL(mock_frontend, setBriskDetectionThreshold(_))
        .Times(1);

    ThreadedKFVio vio(parameters);
}

/* ThreadedKFVio needs IMU data to work
TEST(OkvisVioInterfaces, testDestructionWithImageData)
{
  using namespace okvis;

  MockVioBackendInterface dummy;
  EXPECT_CALL(dummy, optimize(_,_,_))
    .Times(0);

  MockVioFrontendInterface mock_frontend;
  EXPECT_CALL(mock_frontend, detectAndDescribe(_,_,_,_))
    .Times(AtLeast(1));
  EXPECT_CALL(mock_frontend, dataAssociationAndInitialization(_,_,_,_,_,_))
    .Times(Between(0, 10));
  EXPECT_CALL(mock_frontend, propagation(_,_,_,_,_,_,_,_))
    .Times(0);

  okvis::VioParameters parameters;
  parameters.nCameraSystem = TestDataGenerator::getTestCameraSystem(2);

  // start with mock
  ThreadedKFVio vio(parameters, dummy, mock_frontend);
  vio.setBlocking(false);

  double now = okvis::Time::now().toSec();

  // test Observation
  cv::Mat image_cam0;
  cv::Mat image_cam1;
  image_cam0 = cv::imread("testImage.jpg", 0);
  image_cam1 = cv::imread("testImage.jpg", 0);

  if (image_cam0.data == NULL) {
    std::cout << "testImage.jpg not found" << std::endl;
  }

  ASSERT_TRUE(image_cam0.data != NULL);
  ASSERT_TRUE(image_cam1.data != NULL);

  // simulate 100Hz IMU and 10Hz images
  for (int i = 0; i < 3; ++i) {
    vio.addImage(okvis::Time(now), 0, image_cam0);
    vio.addImage(okvis::Time(now), 1, image_cam1);
    now += 0.1;
  }
}
*/

TEST(OkvisVioInterfaces, testDestructionWithIMUData)
{
  using namespace okvis;

  MockVioBackendInterface dummy;
  EXPECT_CALL(dummy, addCamera(_))
    .Times(2);
  EXPECT_CALL(dummy, addImu(_))
    .Times(1);
  EXPECT_CALL(dummy, optimize(_,_,_))
    .Times(0);
  EXPECT_CALL(dummy, currentFrameId())
    .Times(1);
  EXPECT_CALL(dummy, get_T_WS(_,_))
    .Times(1);
  EXPECT_CALL(dummy, setOptimizationTimeLimit(_,_))
    .Times(1);

  MockVioFrontendInterface mock_frontend;
  EXPECT_CALL(mock_frontend, detectAndDescribe(_,_,_,_))
    .Times(0);
  EXPECT_CALL(mock_frontend, propagation(_,_,_,_,_,_,_,_))
    .Times(Between(9, 10));
  EXPECT_CALL(mock_frontend, setBriskDetectionOctaves(_))
    .Times(1);
  EXPECT_CALL(mock_frontend, setBriskDetectionThreshold(_))
    .Times(1);

  okvis::VioParameters parameters;
  parameters.nCameraSystem = TestDataGenerator::getTestCameraSystem(2);
  parameters.publishing.publishImuPropagatedState = true;

  // start with mock
  ThreadedKFVio vio(parameters);
  vio.setBlocking(true);

  double now = okvis::Time::now().toSec();

  // test Observation
  okvis::ImuMeasurement imu_data;
  imu_data.measurement.accelerometers.Zero();
  imu_data.measurement.gyroscopes.Zero();

  for (int j = 0; j < 10; ++j) {
    vio.addImuMeasurement(okvis::Time(now + j * 0.01), imu_data.measurement.accelerometers, imu_data.measurement.gyroscopes);
  }
}
