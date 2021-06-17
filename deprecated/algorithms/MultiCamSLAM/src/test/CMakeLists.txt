add_executable(test_cvFrontEnd1 test_cvFrontEnd1.cpp)
target_link_libraries(test_cvFrontEnd1
FeatureFrontEndCV
${REQUIRED_LIBRARIES}
)

add_executable(test_cvFrontEnd2_optflow test_cvFrontEnd2_optflow.cpp)
target_link_libraries(test_cvFrontEnd2_optflow
FeatureFrontEndCV
${REQUIRED_LIBRARIES}
)
add_executable(test_cvFrontEnd3_createStereoFrame test_cvFrontEnd3_createStereoFrame.cpp)
target_link_libraries(test_cvFrontEnd3_createStereoFrame
    FeatureFrontEndCV
    ${REQUIRED_LIBRARIES}
    )
add_executable(test_FrameManager4_trackLocalFrame test_FrameManager4_trackLocalFrame.cpp)
target_link_libraries(test_FrameManager4_trackLocalFrame
    FeatureFrontEndCV
    FrameManager
    ${REQUIRED_LIBRARIES}
    )
add_executable(test_VO test_VO.cpp)
target_link_libraries(test_VO
    FeatureFrontEndCV
    FrameManager
    ${REQUIRED_LIBRARIES}
    )
add_executable(test_SLAM_simple test_SLAM_simple.cpp)
target_link_libraries(test_SLAM_simple
FeatureFrontEndCV
FrameManager
${REQUIRED_LIBRARIES}
#SLAMOptimizationGraph
IMU_Preint_GTSAM
ReprojectionInfoDatabase
SlidingWindow

)
add_executable(test_SLAM_offline test_SLAM_offline.cpp)
target_link_libraries(test_SLAM_offline
FeatureFrontEndCV
FrameManager
${REQUIRED_LIBRARIES}
#SLAMOptimizationGraph
IMU_Preint_GTSAM
ReprojectionInfoDatabase
SlidingWindow
)

