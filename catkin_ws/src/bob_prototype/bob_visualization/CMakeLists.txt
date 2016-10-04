cmake_minimum_required(VERSION 2.8.3)
project(bob_visualization)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
bob_toolbox
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_visualization
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_visualization
  src/visualization.cpp
  src/visualizer.cpp
  src/bob_toolbox_visualization.cpp
)

target_link_libraries(bob_visualization ${catkin_LIBRARIES})
