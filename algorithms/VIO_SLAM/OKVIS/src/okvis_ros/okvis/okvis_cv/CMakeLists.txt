cmake_minimum_required(VERSION 2.8.11)

project(okvis_cv)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

# require OpenCV
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 

# build the library
add_library(${PROJECT_NAME} STATIC 
  src/CameraBase.cpp
  src/NCameraSystem.cpp
)

# and link it
target_link_libraries(${PROJECT_NAME}  
  PUBLIC okvis_util
  PUBLIC okvis_kinematics 
  PUBLIC okvis_time
  PRIVATE ${OpenCV_LIBRARIES} 
  PRIVATE ${BRISK_LIBRARIES}
)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")

# testing
if(BUILD_TESTS)
  if(APPLE)
    add_definitions(-DGTEST_HAS_TR1_TUPLE=1)
  else()
    add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
  endif(APPLE)
  enable_testing()
  set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
  add_executable(${PROJECT_TEST_NAME} 
    test/runTests.cpp
    test/TestPinholeCamera.cpp
    test/TestFrame.cpp
    test/TestNCameraSystem.cpp
    test/TestMultiFrame.cpp
  )
  target_link_libraries(${PROJECT_TEST_NAME} 
    ${PROJECT_NAME} 
    ${GTEST_LIBRARY}  
    pthread)
  add_test(test ${PROJECT_TEST_NAME})
endif()
