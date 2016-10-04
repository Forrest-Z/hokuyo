cmake_minimum_required(VERSION 2.8.3)
project(bob_core)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_lidar
  bob_control
  bob_toolbox
  bob_grid_map
  bob_config
  bob_coverage
  bob_sensor
  bob_control_handle
  bob_stc
  bob_frontier_exploration
  bob_map_algorithms
  bob_system
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_core
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_core
  src/alternating_exploration_coverage.cpp
  src/area_processor.cpp
  src/area_coverer.cpp
)
