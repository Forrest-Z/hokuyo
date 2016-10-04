cmake_minimum_required(VERSION 2.8.3)
project(bob_high_level)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
#find_package(PkgConfig)
#pkg_check_modules(BFL REQUIRED orocos-bfl)
#
#include_directories(${BFL_INCLUDE_DIRS})
#link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  bob_map_algorithms
  bob_grid_map
  bob_toolbox
  bob_control
  bob_coverage
  bob_config
  bob_slam
  bob_core
  bob_sensor
  bob_control_handle
  bob_boustrophedon
  bob_stc
  bob_wall_following
  bob_frontier_exploration
  bob_navigation
  bob_ros_implementations
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_high_level
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_high_level
  src/demo.cpp
#  src/<other_file>.cpp
)

target_link_libraries(bob_high_level ${catkin_LIBRARIES})
