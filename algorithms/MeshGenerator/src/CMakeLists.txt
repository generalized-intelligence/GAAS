add_library(MeshGenerator MeshGenerator.cpp)
target_link_libraries(MeshGenerator
    ${REQUIRED_LIBRARIES}
)

add_executable(Pipeline Pipeline.cpp)
target_link_libraries(Pipeline
    MeshGenerator
    ${REQUIRED_LIBRARIES}
)
add_executable(ROSNode ROS_node.cpp)
target_link_libraries(ROSNode
    MeshGenerator
    ${REQUIRED_LIBRARIES}
)
