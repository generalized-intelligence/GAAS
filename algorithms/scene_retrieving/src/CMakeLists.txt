add_library(scene_retrieve scene_retrieve.cpp)
target_link_libraries(scene_retrieve
			${REQUIRED_LIBRARIES})
add_executable(demo demo.cpp)
target_link_libraries(demo
			${REQUIRED_LIBRARIES}
)
