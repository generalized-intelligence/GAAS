
add_library(FeatureFrontEndCV FeatureFrontEndCV.cpp)
target_link_libraries(FeatureFrontEndCV
${REQUIRED_LIBRARIES}
)
add_library(FrameManager FrameManager.cpp)
target_link_libraries(FrameManager
    FeatureFrontEndCV
    ${REQUIRED_LIBRARIES}
    )
#add_library(FrameWiseGeometry FrameWiseGeometry.cpp)
#target_link_libraries(FrameWiseGeometry 
#${REQUIRED_LIBRARIES}
#)
add_library(SLAMOptimizationGraph SLAMOptimizationGraph.cpp)
target_link_libraries(SLAMOptimizationGraph
${REQUIRED_LIBRARIES}
FeatureFrontEndCV
FrameManager

)
add_library(IMU_Preint_GTSAM IMU_Preint_GTSAM.cpp)
target_link_libraries(IMU_Preint_GTSAM ${REQUIRED_LIBRARIES}
)
add_library(ReprojectionInfoDatabase ReprojectionInfoDatabase.cpp)
target_link_libraries(ReprojectionInfoDatabase IMU_Preint_GTSAM ${REQUIRED_LIBRARIES})



add_library(SlidingWindow SlidingWindow.cpp)
target_link_libraries(SlidingWindow
${REQUIRED_LIBRARIES}
FeatureFrontEndCV
FrameManager
SLAMOptimizationGraph
IMU_Preint_GTSAM
ReprojectionInfoDatabase
)

add_executable(SLAM_simple SLAM_simple.cpp)
target_link_libraries(SLAM_simple
${REQUIRED_LIBRARIES}
FeatureFrontEndCV
FrameManager
SLAMOptimizationGraph
IMU_Preint_GTSAM
SlidingWindow
)
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} FeatureFrontEndCV FrameManager SLAMOptimizationGraph IMU_Preint_GTSAM ReprojectionInfoDatabase SlidingWindow)
add_subdirectory(test)
