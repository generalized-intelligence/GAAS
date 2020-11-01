cmake_minimum_required(VERSION 2.8.3)
project(velodyne_description)

find_package(catkin REQUIRED)

catkin_package()

# Extract *.tar.gz files in the mesh folder
FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/meshes" "${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.tar.gz")
FOREACH(filename ${files})
  MESSAGE(STATUS "Extracting file: ${filename}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  )
ENDFOREACH(filename)

install(DIRECTORY launch meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
