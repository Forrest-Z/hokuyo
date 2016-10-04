cmake_minimum_required(VERSION 2.8.3)
project(bob_boustrophedon)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_control
  bob_coverage
  bob_lidar
  bob_sensor
  bob_toolbox
  bob_control_handle
  bob_wall_following
  bob_system
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_boustrophedon
  CATKIN_DEPENDS
  bob_control
  bob_coverage
  bob_lidar
  bob_sensor
  bob_toolbox
  bob_control_handle
  bob_wall_following
  bob_system
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_boustrophedon
  src/boustrophedon_task_parameters.cpp
  src/boustrophedon_curve_spaces.cpp
  src/boustrophedon_subtask_executor.cpp
  src/boustrophedon.cpp
  src/boustrophedon_executor.cpp
  src/boustrophedon_condition.cpp
)

target_link_libraries(bob_boustrophedon
  ${catkin_LIBRARIES}
)
