cmake_minimum_required(VERSION 2.8.3)
project(bob_launch)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bop_scripts
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
