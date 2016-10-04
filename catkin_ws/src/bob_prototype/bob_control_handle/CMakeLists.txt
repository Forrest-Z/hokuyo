cmake_minimum_required(VERSION 2.8.3)
project(bob_control_handle)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_control
  bob_sensor
  bob_navigation
  bob_sensor
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES bob_control_handle
 CATKIN_DEPENDS
  bob_control
  bob_sensor
  bob_navigation
  bob_sensor
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_control_handle
  src/control_handle.cpp
)
