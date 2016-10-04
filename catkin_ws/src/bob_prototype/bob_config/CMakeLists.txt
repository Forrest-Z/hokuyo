cmake_minimum_required(VERSION 2.8.3)
project(bob_config)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_config
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include
${catkin_INCLUDE_DIRS})

add_library(bob_config
src/config.cpp
)
