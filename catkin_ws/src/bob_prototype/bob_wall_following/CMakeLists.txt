cmake_minimum_required(VERSION 2.8.3)
project(bob_wall_following)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_config
  bob_control
  bob_lidar
  bob_toolbox
  bob_system
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bob_wall_following
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_wall_following
  src/wall_follow_stuck_condition.cpp
  src/wall_follower.cpp
  src/wall_follow_back_condition.cpp
  src/wall_follow_heading_chooser.cpp
  src/wall_hugger.cpp
  src/wall_hug_command.cpp
  src/wall_follow_overlap_condition.cpp
)

target_link_libraries(bob_wall_following ${catkin_LIBRARIES})
