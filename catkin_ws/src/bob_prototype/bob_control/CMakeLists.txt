cmake_minimum_required(VERSION 2.8.3)
project(bob_control)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  bob_config
  bob_grid_map
  bob_lidar
  bob_sensor
  bob_toolbox
  bob_visualization
  bob_sensor
  bob_system
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_control
  CATKIN_DEPENDS 
  bob_config
  bob_grid_map
  bob_lidar
  bob_sensor
  bob_toolbox
  bob_visualization
  bob_sensor
  bob_system
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(bob_control
  src/simple_commander.cpp
  src/control_result.cpp
  src/controller_runner.cpp
  src/straight_path_executor.cpp
  src/straight_driver.cpp
  src/unsafe_range_calculator.cpp
  src/linear_accelerate_controller.cpp
  src/rotation_controller.cpp
  src/pid_controller.cpp
  src/line_pid_controller.cpp
  src/velocity_repeater.cpp
  src/curve_heading.cpp
  src/curve_to_line_controller.cpp
  src/curve_planner.cpp
  src/curved_path_executor.cpp
  src/curved_subtask_executor.cpp
  src/curve_tools.cpp
  src/robot_remote.cpp
  src/recovery_behavior.cpp

  # conditions
  src/conditions/composite_condition.cpp
  src/conditions/ored_condition.cpp
  src/conditions/anded_condition.cpp
  src/conditions/null_condition.cpp
  src/conditions/proximity_condition.cpp
  src/conditions/half_space_condition.cpp
  src/conditions/front_obstacle_condition.cpp
  src/conditions/distance_robot_direction_condition.cpp
  src/conditions/heading_condition.cpp
  src/conditions/velocity_condition.cpp
  src/conditions/pointing_to_goal_condition.cpp
  src/conditions/distance_condition.cpp
  src/conditions/distance_world_direction_condition.cpp
  src/conditions/function_pointer_condition.cpp
  src/conditions/bumper_condition.cpp
  src/conditions/controller_loop_condition.cpp
  
  # commands
  src/commands/accelerate_command.cpp
  src/commands/track_line_command.cpp
  src/commands/velocity_repeater_command.cpp
  src/commands/heading_rotation_command.cpp
  src/commands/curve_command.cpp
  src/commands/isimple_command.cpp
  src/commands/stop_command.cpp
)


target_link_libraries(bob_control ${catkin_LIBRARIES})
