cmake_minimum_required(VERSION 2.8.3)
project(bob_map_algorithms)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_grid_map
  bob_toolbox
  bob_config
  bob_visualization
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_maps
  CATKIN_DEPENDS  
  bob_config
  bob_visualization
#  DEPENDS system_lib
)

include_directories(include
${catkin_INCLUDE_DIRS}
)


add_library(bob_maps
	src/closest_clear_point.cpp
	src/obstacle_locator.cpp
	src/map_functor.cpp
	src/helper_functions.cpp
)

target_link_libraries(bob_maps ${catkin_LIBRARIES})
