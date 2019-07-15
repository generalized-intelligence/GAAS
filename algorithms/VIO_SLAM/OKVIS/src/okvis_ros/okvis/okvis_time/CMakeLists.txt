cmake_minimum_required(VERSION 2.8.11)

project(okvis_time)

IF(libvisensor_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  MESSAGE(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
ENDIF()

# build the library
add_library(${PROJECT_NAME}
  src/Time.cpp
  src/Duration.cpp)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
