cmake_minimum_required(VERSION 2.8.3)
project(bob_coverage)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  bob_grid_map
  bob_toolbox
  bob_control
  bob_config
  bob_map_algorithms
  bob_sensor
  bob_control_handle
  bob_frontier_exploration
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_coverage
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
${catkin_INCLUDE_DIRS})

add_library(bob_coverage
  src/coverage_tracker.cpp
  src/discrete_area.cpp
  src/discrete_area_shapes.cpp
  src/area_chooser.cpp
  src/wall_chooser.cpp
  src/coverable_area_tracker.cpp
  src/area_tools.cpp
  src/binary_map_area.cpp
  src/visualization.cpp
)
target_link_libraries(bob_coverage
  ${catkin_LIBRARIES} 
)

  # Should move to bob_stc
  #test/stc_path_combiner_test.cpp
  #test/stc_plan_tools_test.cpp


