#######################################
# user config

# Based on the ARM_ARCHITECTURE environment variable
# Set IS_ARM to build on the Odroid board
IF(DEFINED ENV{ARM_ARCHITECTURE})
  MESSAGE("ARM_ARCHITECTURE environment variable set to " $ENV{ARM_ARCHITECTURE})
  SET(IS_ARM TRUE)
ELSE()
  SET(IS_ARM FALSE)
ENDIF()

SET(BUILD_TEST TRUE)

#######################################
SET(PROJECT_NAME fast)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Build type and flags
SET(CMAKE_BUILD_TYPE Release) # Options:  Debug, RelWithDebInfo, Release
SET(CMAKE_CXX_FLAGS "-Wall -Werror ")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
IF(IS_ARM)
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
ENDIF()

# Add Sources
SET(SOURCES  ${PROJECT_SOURCE_DIR}/src/fast_10.cpp
             ${PROJECT_SOURCE_DIR}/src/fast_10_score.cpp
             ${PROJECT_SOURCE_DIR}/src/nonmax_3x3.cpp)
IF(IS_ARM)
#  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
ELSE()
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
ENDIF()

# Add library
INCLUDE_DIRECTORIES(include src)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add Tests
IF(BUILD_TEST)
  MESSAGE("Building Fast Test")
  FIND_PACKAGE(OpenCV REQUIRED)
  ADD_DEFINITIONS(-DTEST_DATA_DIR=\"${PROJECT_SOURCE_DIR}/test/data\")
  ADD_EXECUTABLE(fast_test test/test.cpp)
  TARGET_LINK_LIBRARIES(fast_test ${OpenCV_LIBS} ${PROJECT_NAME})
ENDIF()


################################################################################
# Create the fastConfig.cmake file for other cmake projects.
GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
SET(fast_LIBRARIES ${FULL_LIBRARY_NAME} )
SET(fast_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
SET(fast_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/fastConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/fastConfig.cmake @ONLY IMMEDIATE )
export( PACKAGE fast )

INSTALL(DIRECTORY include/fast DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
