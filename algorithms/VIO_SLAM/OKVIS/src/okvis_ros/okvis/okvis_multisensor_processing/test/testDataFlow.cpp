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
using ::testing::AnyNumber;
using ::testing::Between;
using ::testing::Return;
using ::testing::_;

TEST(OkvisVioInterfaces, testDataFlow)
{
  using namespace okvis;

  okvis::VioParameters parameters;
  parameters.nCameraSystem = TestDataGenerator::getTestCameraSystem(2);
  parameters.visualization.displayImages = false;
  parameters.imu.a_max = 1;
  parameters.imu.g_max = 1;
  parameters.optimization.numImuFrames = 2;
  parameters.publishing.publishImuPropagatedState = true;

  // configure mock object
  MockVioBackendInterface dummy;
  EXPECT_CALL(dummy, optimize(_,_,_))
    .Times(Between(5, 10));
  EXPECT_CALL(dummy, get_T_WS(_,_))
    .Times(Between(12, 21));        // 1 per matching, 1 per optimization and in destructor
  EXPECT_CALL(dummy, addCamera(_))
    .Times(2);
  EXPECT_CALL(dummy, addImu(_))
    .Times(1);
  EXPECT_CALL(dummy, addStates(_,_,_))
    .Times(Between(5, 10));
  EXPECT_CALL(dummy, applyMarginalizationStrategy(_,_,_))
    .Times(Between(5,10));
  EXPECT_CALL(dummy, setOptimizationTimeLimit(_,_))
    .Times(1);

  EXPECT_CALL(dummy, multiFrame(_))
    .Times(AnyNumber());
  EXPECT_CALL(dummy, getSpeedAndBias(_,_,_))
    .Times(AnyNumber());
  EXPECT_CALL(dummy, numFrames())
    .Times(AnyNumber());

  ON_CALL(dummy, numFrames())
    .WillByDefault(Return(1));
  ON_CALL(dummy, addStates(_,_,_))
    .WillByDefault(Return(true));
  // to circumvent segfault
  ON_CALL(dummy, multiFrame(_))
    .WillByDefault(Return(std::shared_ptr<okvis::MultiFrame>(new okvis::MultiFrame(parameters.nCameraSystem,
                                                                                   okvis::Time::now()))));

  MockVioFrontendInterface mock_frontend;
  EXPECT_CALL(mock_frontend, detectAndDescribe(_,_,_,_))
    .Times(Between(18, 20));
  EXPECT_CALL(mock_frontend, dataAssociationAndInitialization(_,_,_,_,_,_))
    .Times(Between(7, 10));
  EXPECT_CALL(mock_frontend, propagation(_,_,_,_,_,_,_,_))
    .Times(Between(99, 100));
  EXPECT_CALL(mock_frontend, setBriskDetectionOctaves(_))
      .Times(1);
  EXPECT_CALL(mock_frontend, setBriskDetectionThreshold(_))
      .Times(1);

  // start with mock
  ThreadedKFVio vio(parameters);
  vio.setBlocking(true);

  double now = okvis::Time::now().toSec();

  // test Observation
  cv::Mat image_cam;
  image_cam = cv::imread("testImage.jpg", 0);

  if (image_cam.data == NULL) {
    std::cout << "testImage.jpg not found" << std::endl;
  }

  ASSERT_TRUE(image_cam.data != NULL);

  okvis::ImuMeasurement imu_data;
  imu_data.measurement.accelerometers.Zero();  imu_data.measurement.gyroscopes.Zero();

  // simulate 100Hz IMU and 10Hz images
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 5; ++j) {
      vio.addImuMeasurement(okvis::Time(now), imu_data.measurement.accelerometers, imu_data.measurement.gyroscopes);
      now += 0.01;
    }
    vio.addImage(okvis::Time(now), 0, image_cam);
    vio.addImage(okvis::Time(now), 1, image_cam);
    for (int j = 0; j < 5; ++j) {
      vio.addImuMeasurement(okvis::Time(now), imu_data.measurement.accelerometers, imu_data.measurement.gyroscopes);
      now += 0.01;
    }
  }
}
