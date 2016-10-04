cmake_minimum_required(VERSION 2.8.3)
project(bob_frontier_exploration)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_config
  bob_grid_map
  bob_map_algorithms
  bob_toolbox
  bob_control
  bob_navigation
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_frontier_exploration
  CATKIN_DEPENDS 
  bob_config
  bob_grid_map
  bob_map_algorithms
  bob_toolbox
  bob_control
  bob_navigation
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_frontier_exploration
  src/wavefront_frontier_locator.cpp 
  src/frontier_cloud.cpp
  src/frontier_tracker.cpp
  src/frontier_condition.cpp
  src/frontier_chooser.cpp
#  src/frontier_explorer.cpp
)

target_link_libraries(bob_frontier_exploration ${catkin_LIBRARIES})
