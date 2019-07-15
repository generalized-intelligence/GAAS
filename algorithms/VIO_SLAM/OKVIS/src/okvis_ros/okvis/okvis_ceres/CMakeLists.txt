cmake_minimum_required(VERSION 2.8.11)
project(okvis_ceres)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 
               
# build the library 
add_library(${PROJECT_NAME} 
  src/PoseParameterBlock.cpp
  src/SpeedAndBiasParameterBlock.cpp
  src/HomogeneousPointParameterBlock.cpp
  src/HomogeneousPointLocalParameterization.cpp
  src/PoseLocalParameterization.cpp
  src/ImuError.cpp
  src/PoseError.cpp
  src/RelativePoseError.cpp
  src/SpeedAndBiasError.cpp
  src/IdProvider.cpp
  src/Map.cpp
  src/MarginalizationError.cpp
  src/HomogeneousPointError.cpp
  src/Estimator.cpp
  src/LocalParamizationAdditionalInterfaces.cpp
  include/okvis/Estimator.hpp
  include/okvis/ceres/CeresIterationCallback.hpp
)

# and link it
target_link_libraries(${PROJECT_NAME} 
  PUBLIC okvis_util
  PUBLIC okvis_cv 
  PUBLIC okvis_common
  PRIVATE ${CERES_LIBRARIES} 
  PRIVATE ${OpenCV_LIBRARIES} 
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
    test/test_main.cpp
    test/TestEstimator.cpp
    test/TestHomogeneousPointError.cpp
    test/TestReprojectionError.cpp
    test/TestImuError.cpp
    test/TestMap.cpp
    test/TestMarginalization.cpp
  )
  target_link_libraries(${PROJECT_TEST_NAME} 
    ${PROJECT_NAME} 
    ${GTEST_LIBRARY}  
    pthread)
  add_test(test ${PROJECT_TEST_NAME})
endif()
