cmake_minimum_required(VERSION 2.8.3)
project(bob_system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
bob_toolbox
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_system
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_system
  src/system_utilities.cpp
  src/periodic_threader.cpp
)

target_link_libraries(bob_system 
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)
