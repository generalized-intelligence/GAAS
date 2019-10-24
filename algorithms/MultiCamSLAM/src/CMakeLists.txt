
add_library(FeatureFrontEndCV FeatureFrontEndCV.cpp)
target_link_libraries(FeatureFrontEndCV
${REQUIRED_LIBRARIES}
)

#add_library(FrameWiseGeometry FrameWiseGeometry.cpp)
#target_link_libraries(FrameWiseGeometry 
#${REQUIRED_LIBRARIES}
#)


add_subdirectory(test)
