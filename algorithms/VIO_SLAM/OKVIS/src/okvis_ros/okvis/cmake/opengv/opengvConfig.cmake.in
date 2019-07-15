# - Config file for the OPENGV package
# It defines the following variables
#  OPENGV_INCLUDE_DIRS - include directories for FooBar
#  OPENGV_LIBRARIES    - libraries to link against
#  OPENGV_EXECUTABLE   - the opengv executable - none available

# Compute paths
get_filename_component(OPENGV_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET opengv AND NOT OPENGV_BINARY_DIR)
  include("${OPENGV_CMAKE_DIR}/opengvTargets.cmake")
endif()

set(OPENGV_LIBRARIES opengv)
set(OPENGV_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")
