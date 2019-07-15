cmake_minimum_required(VERSION 2.8.11)

project(okvis)
# The version number.
set(OKVIS_MAJOR_VERSION 1)
set(OKVIS_MINOR_VERSION 1)
set(OKVIS_PATCH_VERSION 3)
set(OKVIS_VERSION
  ${OKVIS_MAJOR_VERSION}.${OKVIS_MINOR_VERSION}.${OKVIS_PATCH_VERSION})

# options for build configuration
option (USE_SYSTEM_BRISK
        "Use brisk via find_package rather than downloading it as part of okvis" OFF) 
option (USE_SYSTEM_CERES
        "Use ceres via find_package rather than downloading it as part of okvis" OFF) 
option (BUILD_APPS
        "Builds a demo app (which requires boost)" ON) 
option (BUILD_TESTS
        "Builds all gtests" OFF) 
SET(N_CORES 3 CACHE STRING "Using N number of cores for parallel build")

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/okvis)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")
 
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
else()
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}'")
endif()

if(APPLE)
  # The clang compiler (on osx) is somehow much more strict
  # than the compilers on ubuntu and so this does not seem
  # possible on OSX just yet.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
  message(STATUS "ARM processor detected, will attempt to use NEON.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
else()
  message(STATUS "Assuming SSE instructions available.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
endif()

get_directory_property(defs COMPILE_DEFINITIONS)

# EXTERNAL DEPENDENCIES
include(ExternalProject)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include_directories(${CMAKE_BINARY_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/lib)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

# libvisensor
find_package(VISensor)

if(VISENSORDRIVER_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  set(LIBVISENSOR TRUE)
  message(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
  include_directories(${VISensorDriver_INCLUDE_DIR})
  # install find script
  install(FILES ${PROJECT_SOURCE_DIR}/cmake/FindVISensor.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake)
else()
  message(STATUS "libvisensor not found")
endif()



# glog
find_package(Glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS}) 

# brisk
if(${USE_SYSTEM_BRISK})
  find_package( brisk 2 REQUIRED )
  include_directories(BEFORE ${BRISK_INCLUDE_DIRS})
  message(STATUS "Using system brisk. Found at ${BRISK_INCLUDE_DIRS}.")
else()
  ExternalProject_Add(brisk_external
    URL "https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.3.zip"    
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/brisk
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
  )
  add_library(brisk STATIC IMPORTED)
  add_library(agast STATIC IMPORTED)
  set(BRISK_LIBRARIES brisk agast)
  set_target_properties(brisk PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libbrisk.a)
  set_target_properties(agast PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libagast.a)
  add_dependencies(brisk brisk_external)
  add_dependencies(agast brisk_external)

  # install it once built
  install(FILES ${CMAKE_BINARY_DIR}/lib/libbrisk.a ${CMAKE_BINARY_DIR}/lib/libagast.a 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/include/brisk/ 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/brisk/)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/CMake/brisk/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake/brisk/)
endif()

# ceres
find_package(SuiteSparse REQUIRED QUIET) #will be needed in any case...
find_package(CXSparse QUIET) #optional
if(${USE_SYSTEM_CERES})
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  find_package( Ceres 1.9 REQUIRED )
  include_directories(BEFORE ${CERES_INCLUDE_DIRS})
  message(STATUS "Using system Ceres.")
else()
  set(CERES_VERSION 1.9.0)
  set(CERES_TAG 7c57de5080c9f5a4f067e2d20b5f33bad5b1ade6) #version 1.9. Newer ones would need adaptation with ceres::ParameterBlock
  ExternalProject_Add(ceres_external
    GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
    GIT_TAG ${CERES_TAG}
    UPDATE_COMMAND ""
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    CMAKE_ARGS 
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> 
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} 
      -DBUILD_EXAMPLES:BOOL=OFF
      -DBUILD_TESTING:BOOL=OFF
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ceres
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
  )

  add_library(ceres UNKNOWN IMPORTED)
  # Heuristic for determining LIB_SUFFIX from ceres
  set(CERES_LIB_PREFIX "lib")
  if (CMAKE_SYSTEM_NAME MATCHES "Linux" AND
    NOT DEFINED LIB_SUFFIX AND
    NOT CMAKE_CROSSCOMPILING AND
    CMAKE_SIZEOF_VOID_P EQUAL "8" AND
    NOT EXISTS "/etc/debian_version" AND
    NOT EXISTS "/etc/arch-release")
   set(CERES_LIB_PREFIX "lib64")
  endif ()

  # to find it from 3rd party software, since not added to registry:
  set(OKVIS_CERES_CONFIG "${CMAKE_BINARY_DIR}/share/Ceres/")

  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set_property(TARGET ceres PROPERTY IMPORTED_LOCATION
               ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres-debug.a )
  else () 
    set_property(TARGET ceres PROPERTY IMPORTED_LOCATION
               ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres.a )
  endif ()
   
  find_package(OpenMP QUIET)
  if (OPENMP_FOUND)
    message(STATUS "OpenMP found")
    add_definitions(-DUSE_OPENMP)
    set(CERES_LIBRARIES ceres gomp ${SUITESPARSE_LIBRARIES} ${CXSPARSE_LIBRARIES} ${Eigen_LIBS} ${GLOG_LIBRARY}) #ceres won't export these
  else ()
    message(STATUS "OpenMP NOT found")
    set(CERES_LIBRARIES ceres ${SUITESPARSE_LIBRARIES} ${CXSPARSE_LIBRARIES}  ${Eigen_LIBS} ${GLOG_LIBRARY}) #ceres won't export these
  endif ()
  add_dependencies(${CERES_LIBRARIES} ceres_external)

  # install it once built
  install(FILES ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres.a 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/include/ceres/ 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ceres/)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/share/Ceres/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/Ceres/)
endif()

# OpenGV
set(CMAKE_CXX_FLAGS_WSUPPRESS 
      "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wno-maybe-uninitialized -Wno-sign-compare -Wno-unused-but-set-variable -Wno-unused-variable -Wno-pedantic")
message(STATUS "NOTE: Suppressing some warnings when compiling OpenGV.")
ExternalProject_Add(opengv_external
    GIT_REPOSITORY https://github.com/laurentkneip/opengv
    GIT_TAG cc32b16281aa6eab67cb28a61cf87a2a5c2b0961
    UPDATE_COMMAND ""
    PATCH_COMMAND # forcing static libs
        COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/cmake/opengv/CMakeLists.txt ${CMAKE_CURRENT_BINARY_DIR}/opengv/src/opengv/CMakeLists.txt
        COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/cmake/opengv/opengvConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/opengv/src/opengv/opengvConfig.cmake.in
        COMMENT "Forcing our own CMakeLists.txt to build OpenGV (static library support)."
    CMAKE_ARGS 
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> 
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE} 
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_WSUPPRESS} # suppress warnings...
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/opengv
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opengv/src/opengv
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
)
add_library(opengv STATIC IMPORTED)
set(OpenGV_LIBRARIES opengv)
set_target_properties(opengv PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libopengv.a)
add_dependencies(${OpenGV_LIBRARIES} opengv_external)

# install it once built
install(FILES ${CMAKE_BINARY_DIR}/lib/libopengv.a 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY ${CMAKE_BINARY_DIR}/include/opengv/ 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/opengv/)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/CMake/opengv/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake/opengv/)

if(${BUILD_TESTS})
  # gmock
  # http://stackoverflow.com/questions/9689183/cmake-googletest
  ExternalProject_Add(
    googlemock
    URL https://googlemock.googlecode.com/files/gmock-1.7.0.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gmock
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND ""
  )
  ExternalProject_Get_Property(googlemock source_dir)
  set(GMOCK_INCLUDE_DIR ${source_dir}/include)
  set(GTEST_INCLUDE_DIR ${source_dir}/gtest/include)
  include_directories(BEFORE ${GMOCK_INCLUDE_DIR})
  include_directories(BEFORE ${GTEST_INCLUDE_DIR})
  ExternalProject_Get_Property(googlemock binary_dir)
  set(GMOCK_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gmock.a)
  set(GMOCK_LIBRARY gmock)
  add_library(${GMOCK_LIBRARY} UNKNOWN IMPORTED)
  set_property(TARGET ${GMOCK_LIBRARY} PROPERTY IMPORTED_LOCATION
                  ${GMOCK_LIBRARY_PATH} )
  add_dependencies(${GMOCK_LIBRARY} googlemock)
  set(GTEST_LIBRARY_PATH ${binary_dir}/gtest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a)
  set(GTEST_LIBRARY gtestlocal)
  add_library(${GTEST_LIBRARY} UNKNOWN IMPORTED)
  set_property(TARGET ${GTEST_LIBRARY} PROPERTY IMPORTED_LOCATION
                  ${GTEST_LIBRARY_PATH})
  add_dependencies(${GTEST_LIBRARY} googlemock)
endif()

# BUILD LOCAL DEPENDENCIES
include_directories(okvis_util/include)
add_subdirectory(okvis_util)

include_directories(okvis_kinematics/include)
add_subdirectory(okvis_kinematics)
add_dependencies(okvis_kinematics okvis_util)

include_directories(okvis_time/include)
add_subdirectory(okvis_time)
add_dependencies(okvis_time okvis_util)

include_directories(okvis_cv/include)
add_subdirectory(okvis_cv)
add_dependencies(okvis_cv brisk okvis_util)

include_directories(okvis_common/include)
add_subdirectory(okvis_common)

include_directories(okvis_ceres/include)
add_subdirectory(okvis_ceres)
add_dependencies(okvis_ceres ceres okvis_util)

include_directories(okvis_timing/include)
add_subdirectory(okvis_timing)

include_directories(okvis_matcher/include)
add_subdirectory(okvis_matcher)
add_dependencies(okvis_matcher okvis_util)

include_directories(okvis_frontend/include)
add_subdirectory(okvis_frontend)
add_dependencies(okvis_frontend opengv okvis_util)

include_directories(okvis_multisensor_processing/include)
add_subdirectory(okvis_multisensor_processing)
add_dependencies(okvis_multisensor_processing okvis_util)

# also build the apps
if(BUILD_APPS)
  FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS}) 
  add_executable(okvis_app_synchronous okvis_apps/src/okvis_app_synchronous.cpp)
  target_link_libraries(okvis_app_synchronous 
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv 
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_matcher
    okvis_frontend 
    okvis_multisensor_processing
    pthread 
    ${Boost_LIBRARIES}
  )
  if(${VISENSORDRIVER_FOUND})
    target_link_libraries(okvis_app_synchronous ${VISensorDriver_LIBRARY})
  endif()
  install(TARGETS okvis_app_synchronous
    # IMPORTANT: Add the executable to the "export-set"
    EXPORT okvisTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endif()

# installation is invoked in the individual modules...
export (TARGETS 
    okvis_app_synchronous
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv 
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_matcher
    okvis_frontend 
    okvis_multisensor_processing 
    FILE "${PROJECT_BINARY_DIR}/okvisTargets.cmake")
export ( PACKAGE okvis )

# Create the okvisConfig.cmake and okvisConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS 
  "${PROJECT_SOURCE_DIR}/okvis_util/include"
  "${PROJECT_SOURCE_DIR}/okvis_kinematics/include"
  "${PROJECT_SOURCE_DIR}/okvis_time/include"
  "${PROJECT_SOURCE_DIR}/okvis_cv/include"
  "${PROJECT_SOURCE_DIR}/okvis_common/include"
  "${PROJECT_SOURCE_DIR}/okvis_ceres/include"
  "${PROJECT_SOURCE_DIR}/okvis_timing/include"
  "${PROJECT_SOURCE_DIR}/okvis_matcher/include"
  "${PROJECT_SOURCE_DIR}/okvis_frontend/include"
  "${PROJECT_SOURCE_DIR}/okvis_multisensor_processing/include"
  "${PROJECT_BINARY_DIR}"
)
configure_file(cmake/okvisConfig.cmake.in
  "${PROJECT_BINARY_DIR}/okvisConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${OKVIS_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/okvisConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/okvisConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/okvisConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/okvisConfigVersion.cmake" @ONLY)
 
# Install the okvisConfig.cmake and okvisConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/okvisConfig.cmake"
  "${PROJECT_BINARY_DIR}/okvisConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")
 
# Install the export set for use with the install-tree
install(EXPORT okvisTargets DESTINATION
  "${INSTALL_CMAKE_DIR}")
