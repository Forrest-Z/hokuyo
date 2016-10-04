cmake_minimum_required(VERSION 2.8.3)
project(bob_stc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_coverage
  bob_grid_map
  bob_toolbox
  bob_control
  bob_config
  bob_map_algorithms
  bob_sensor
  bob_control_handle
  bob_navigation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_stc
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_stc
  src/stc_path_combiner.cpp
  src/stc_hamiltonian.cpp 
  src/stc.cpp
  src/stc_plan_tools.cpp
  src/stc_plan_executor.cpp
  src/stc_planner.cpp
  src/grid_generator.cpp
  src/grid_map.cpp
  src/grid_point.cpp 
  src/grid_set.cpp 
  src/grid.cpp 
  src/grid_tile_type.cpp
  src/spanning_tree_grid_node.cpp 
  src/spanning_tree_grid_depth_return_iterator.cpp 
  src/spanning_tree_grid_depth_iterator.cpp 
  src/spanning_tree_grid.cpp 
  src/optimal_stc_generator.cpp
  src/arbitrary_tree_builder.cpp
  src/low_bend_tree_builder.cpp
  src/visualization.cpp
  src/easy_print.cpp
)

target_link_libraries(bob_stc
  ${catkin_LIBRARIES} 
)
