cmake_minimum_required(VERSION 2.8.3)
project(bob_navigation)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_control
  bob_grid_map
  bob_map_algorithms
  bob_toolbox
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_navigation
  CATKIN_DEPENDS 
  bob_control
  bob_grid_map
  bob_map_algorithms
  bob_toolbox
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_navigation
src/path_splitter.cpp
src/navigation_manager.cpp
src/dijkstra_planner.cpp
)

target_link_libraries(bob_navigation ${catkin_LIBRARIES})
