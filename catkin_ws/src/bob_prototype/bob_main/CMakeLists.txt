cmake_minimum_required(VERSION 2.8.3)
project(bob_main)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
bob_control
bob_grid_map
bob_freertos_implementations
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS 
#  LIBRARIES bob_main
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_library(bob_main
#  src/<some_file>.cpp
#  src/<other_file>.cpp
#)

#target_link_libraries(bob_main ${catkin_LIBRARIES})

add_executable(motor_test src/motor_test.cpp)

target_link_libraries(motor_test
  ${Boost_LIBRARIES}
${catkin_LIBRARIES}
)
