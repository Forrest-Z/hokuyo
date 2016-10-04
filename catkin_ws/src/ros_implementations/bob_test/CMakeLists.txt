cmake_minimum_required(VERSION 2.8.3)
project(bob_test)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_map_algorithms
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  bob_tf
  bob_grid_map
  pluginlib
  bob_toolbox
  bob_control
  bob_coverage
  bob_config
  bob_stc
  bob_slam
  bob_boustrophedon
  bob_control_handle
  bob_sensor
  bob_wall_following
  bob_frontier_exploration
  bob_navigation
  bob_visualization
  bob_ros_implementations
)

catkin_package(
	INCLUDE_DIRS include
	#  LIBRARIES bob_test
	#  CATKIN_DEPENDS other_catkin_pkg
	#  DEPENDS system_lib
	)

include_directories(include
	${catkin_INCLUDE_DIRS}
	)

add_library(bob_test
	src/example_grid_sets.cpp
	src/example_stc_plans.cpp
	src/stc_test_plans.cpp
	src/stc_tester.cpp
	)
target_link_libraries(bob_test
${catkin_LIBRARIES}
)

add_executable(slam src/standalone_slam.cpp)

add_executable(stc_test src/stc_test.cpp)

##add_executable(grid_test src/grid_test.cpp)
##
##add_executable(move_base_test src/move_base_test.cpp)
##
add_executable(path_test src/path_test.cpp)

add_executable(transform_test src/transform_test.cpp)

##add_executable(lidar_listener_test src/lidar_listener_test.cpp)
##
##add_executable(frontier_test src/frontier_test.cpp)
##
add_executable(marker_test src/marker_test.cpp)

add_executable(wall_following_test src/wall_following_test.cpp)

add_executable(curve_test src/curve_test.cpp)

##add_executable(config_test src/config_test.cpp)
##
add_executable(obstacle_detection src/obstacle_detection.cpp)

add_executable(passable_range_test src/passable_range_test.cpp)

add_executable(frontier_detection src/frontier_detection.cpp)

add_executable(empty src/empty_node.cpp)

add_executable(wavefront_test src/wavefront_test.cpp)

add_executable(boustrophedon src/boustrophedon.cpp)

add_executable(commands_test src/commands_test.cpp)

add_executable(arm_expander_test src/arm_expander_test.cpp)

add_executable(map_functor_test src/map_functor_test.cpp)

add_executable(curve_heading_test src/curve_heading_test.cpp)

add_executable(robot_remote_test src/robot_remote_test.cpp)

add_executable(discrete_area_test src/discrete_area_test.cpp)

add_executable(navigation_test src/navigation_test.cpp)

add_executable(lidar_test src/lidar_test.cpp)

target_link_libraries(slam
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(stc_test
${catkin_LIBRARIES}
bob_test
)

##target_link_libraries(grid_test
##${catkin_LIBRARIES}
##bob_test
##)
##
##target_link_libraries(lidar_listener_test
##  ${catkin_LIBRARIES}
##  bob_test
##)
##
##target_link_libraries(move_base_test
##  ${catkin_LIBRARIES}
##bob_test
##
##)
##

target_link_libraries(transform_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(path_test
${catkin_LIBRARIES}
bob_test
)

##target_link_libraries(frontier_test
##  ${catkin_LIBRARIES}
##bob_test
##
##)
##

target_link_libraries(curve_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(marker_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(wall_following_test
${catkin_LIBRARIES}
bob_test
)

##target_link_libraries(config_test
##  ${catkin_LIBRARIES}
##bob_test
##
##)
##
target_link_libraries(obstacle_detection
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(passable_range_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(frontier_detection
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(empty
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(wavefront_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(boustrophedon
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(commands_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(arm_expander_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(map_functor_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(discrete_area_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(curve_heading_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(robot_remote_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(navigation_test
${catkin_LIBRARIES}
bob_test
)

target_link_libraries(lidar_test
${catkin_LIBRARIES}
bob_test
)
