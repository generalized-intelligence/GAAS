add_library(controller
        controller.h
        controller.cpp
        )

add_executable(controller_node controller_node.cpp controller.cpp)
target_link_libraries(controller_node
        ${controller}
        ${REQUIRED_LIBRARIES}
        )

