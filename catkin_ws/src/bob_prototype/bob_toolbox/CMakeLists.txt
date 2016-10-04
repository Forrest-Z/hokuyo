cmake_minimum_required(VERSION 2.8.3)
project(bob_toolbox)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
)

find_package(Boost REQUIRED COMPONENTS thread system
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_toolbox
  CATKIN_DEPENDS 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_toolbox
  src/line_geometry.cpp
  src/direction.cpp 
  src/world_point_shapes.cpp
  src/eight_direction.cpp
  src/map_half_space.cpp
  src/map_multi_space.cpp
  src/world_point.cpp
  src/stat.cpp
  src/angular_range.cpp
  src/easy_print.cpp
  src/low_pass_filter.cpp
  src/simple_patterns.cpp
  src/point_geometry.cpp
  src/matrix_operations.cpp
  src/basic_algebra.cpp
)

target_link_libraries(bob_toolbox ${catkin_LIBRARIES})
