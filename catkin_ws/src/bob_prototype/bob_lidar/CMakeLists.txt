cmake_minimum_required(VERSION 2.8.3)
project(bob_lidar)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_toolbox
  bob_config
  bob_grid_map
  bob_sensor
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_lidar
  CATKIN_DEPENDS 
  bob_toolbox
  bob_config
  bob_grid_map
  bob_sensor
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_lidar
  src/util_functions.cpp
  src/passable_range.cpp
  src/lidar_proximity_sensor.cpp
)

target_link_libraries(bob_lidar
  ${catkin_LIBRARIES}
)
