


set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(nlohmann_json)

add_library(scene_retrieve scene_retrieve.cpp)

link_libraries("${PROJECT_SOURCE_DIR}/../loop_closing/DBow3/build/libloopclosingmanager.so")
target_link_libraries(scene_retrieve
			${REQUIRED_LIBRARIES}
			#../../loop_closing/DBow3/build/
			#libloopclosingmanager.so
			)

#add_executable(demo demo.cpp)

#target_link_libraries(demo
#			${REQUIRED_LIBRARIES}
#)

add_executable(MakeSceneFromOpenSfMModel 
                        MakeSceneFromOpenSfMModel.cpp)

target_link_libraries(MakeSceneFromOpenSfMModel
                      ${REQUIRED_LIBRARIES}
                      scene_retrieve
                      )

target_link_libraries(MakeSceneFromOpenSfMModel PRIVATE nlohmann_json::nlohmann_json)
