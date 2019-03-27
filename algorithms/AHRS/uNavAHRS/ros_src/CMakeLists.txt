

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    visualization_msgs
    message_filters

)
find_package(Boost COMPONENTS serialization wserialization system REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(ros_main
               ros_api_main.cpp
)
target_link_libraries(ros_main
    uNavAHRS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)





