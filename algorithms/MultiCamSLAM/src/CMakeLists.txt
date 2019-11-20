
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

add_executable(SLAM_simple SLAM_simple.cpp)
target_link_libraries(SLAM_simple
${REQUIRED_LIBRARIES}
FeatureFrontEndCV
FrameManager
SLAMOptimizationGraph
)
add_subdirectory(test)
