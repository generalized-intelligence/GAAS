


set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(nlohmann_json)

add_library(scene_retrieve scene_retrieve.cpp)
target_link_libraries(scene_retrieve
			${REQUIRED_LIBRARIES})
add_executable(demo demo.cpp)
target_link_libraries(demo
			${REQUIRED_LIBRARIES}
)
add_executable(MakeSceneFromOpenSfMModel 
                        MakeSceneFromOpenSfMModel.cpp)
target_link_libraries(MakeSceneFromOpenSfMModel
                        ${REQUIRED_LIBRARIES}
                        PRIVATE nlohmann_json::nlohmann_json
)
