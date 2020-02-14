cmake_minimum_required(VERSION 2.6)
project(gaas_planner)

set(CMAKE_CXX_STANDARD 11)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(gaas_planner main.cpp)

install(TARGETS gaas_planner RUNTIME DESTINATION bin)
