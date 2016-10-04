cmake_minimum_required(VERSION 2.8.3)
project(bob_ros_implementations)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  kobuki_msgs
  bob_toolbox
  bob_tf
  bob_sensor
  bob_lidar
  bob_control
  bob_slam
  bob_control_handle
  bob_system
  geometry_msgs
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_ros_implementations
  CATKIN_DEPENDS 
  kobuki_msgs
  bob_toolbox
  bob_tf
  bob_sensor
  bob_lidar
  bob_control
  bob_slam
  bob_control_handle
  bob_system
  geometry_msgs
  visualization_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_ros_implementations
  src/ros_bumper_handle.cpp
  src/ros_transform_handle.cpp
  src/ros_sensor_handle.cpp
  src/ros_odometry_handle.cpp
  src/ros_transform_notifier.cpp
  src/ros_velocity_publisher.cpp
  src/ros_system_handle.cpp
  src/ros_spinner.cpp
  src/ros_lidar_listener.cpp
  src/ros_lidar_handle.cpp
  src/ros_map_listener.cpp
  src/ros_map_sender.cpp
  src/ros_system_utilities.cpp
  src/from_ros.cpp
  src/ros_rate_timer.cpp
  src/ros_timer.cpp
  src/grid_map_visualization.cpp
  src/ros_visualizer.cpp
  src/msg_building.cpp
  src/ros_lidar_processor.cpp
  src/ros_initializer.cpp
  src/lidar_util.cpp
  src/ros_limited_lidar_handle.cpp
  src/configured_system_handles.cpp
)

target_link_libraries(bob_ros_implementations ${catkin_LIBRARIES})
