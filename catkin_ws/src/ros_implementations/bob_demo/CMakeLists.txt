cmake_minimum_required(VERSION 2.8.3)
project(bob_demo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bob_map_algorithms
  rospy
  bob_grid_map
  pluginlib
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
  bob_high_level
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bob_demo
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#include_directories(include
#  ${catkin_INCLUDE_DIRS}
#)

#add_library(bob_demo
#  src/demo.cpp
#)
#target_link_libraries(bob_demo ${catkin_LIBRARIES})

include_directories(
	${catkin_INCLUDE_DIRS}
	)

add_executable(demo src/demo_node.cpp)

target_link_libraries(demo
  ${catkin_LIBRARIES}
)

add_executable(demo_limited_lidar src/demo_limited_lidar.cpp)

target_link_libraries(demo_limited_lidar
  ${catkin_LIBRARIES}
)
