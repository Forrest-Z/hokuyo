cmake_minimum_required(VERSION 2.8.3)
project(bob_freertos_implementations)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
bob_toolbox
bob_sensor
bob_lidar
bob_control
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_freertos_implementations
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_freertos_implementations
  src/freertos_velocity_publisher.cpp
)

target_link_libraries(bob_freertos_implementations ${catkin_LIBRARIES})
