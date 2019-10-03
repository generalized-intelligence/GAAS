set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(nlohmann_json)

add_library(scene_retrieve scene_retrieve.cpp)

link_libraries("${PROJECT_SOURCE_DIR}/../loop_closing/DBow3/build/libloopclosingmanager.so")
target_link_libraries(scene_retrieve
			${REQUIRED_LIBRARIES}
			)


add_definitions(${PCL_DEFINITIONS})

add_executable(MakeSceneFromOpenSfMModel 
                        MakeSceneFromOpenSfMModel.cpp)
target_link_libraries(MakeSceneFromOpenSfMModel
                      ${REQUIRED_LIBRARIES}
                      scene_retrieve
                      )
target_link_libraries(MakeSceneFromOpenSfMModel PRIVATE nlohmann_json::nlohmann_json)


add_executable(demo_stereo
               demo_stereo_main.cpp)
target_link_libraries(demo_stereo
			${REQUIRED_LIBRARIES}
			scene_retrieve)

add_executable(demo_mono
               demo_mono_main.cpp ../include/cv_helper.h)
target_link_libraries(demo_mono
			${REQUIRED_LIBRARIES}
			scene_retrieve)


add_library(multi_scene_retriever multi_scene_retriever.cpp)
target_link_libraries(multi_scene_retriever PRIVATE nlohmann_json::nlohmann_json
                       ${REQUIRED_LIBRARIES}
                       scene_retrieve
			)


add_executable(scene_retrieving_ros scene_retrieving_ros.cpp)
target_link_libraries(scene_retrieving_ros PRIVATE nlohmann_json::nlohmann_json
                       ${REQUIRED_LIBRARIES}
                       scene_retrieve
			)
add_executable(ros_global_optimization ros_global_optimization.cpp)
target_link_libraries(ros_global_optimization PRIVATE nlohmann_json::nlohmann_json
                        ${REQUIRED_LIBRARIES}
                        scene_retrieve
)

add_executable(controller_node controller_node.cpp controller.cpp )
target_link_libraries(controller_node
		scene_retrieve
		${REQUIRED_LIBRARIES}
		)



