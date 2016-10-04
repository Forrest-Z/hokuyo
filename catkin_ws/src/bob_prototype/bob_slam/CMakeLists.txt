cmake_minimum_required(VERSION 2.8.3)
project(bob_slam)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
#find_package(PkgConfig)
#pkg_check_modules(BFL REQUIRED orocos-bfl)

#include_directories(${BFL_INCLUDE_DIRS})
#link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
	bob_config
	bob_toolbox
	bob_lidar
	bob_grid_map
	bob_system
	bob_visualization
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bob_slam_lib
  CATKIN_DEPENDS 
	bob_config
	bob_toolbox
	bob_lidar
	bob_grid_map
	bob_system
	bob_visualization
)

add_library(bob_slam_lib
	src/slam_processor.cpp
	src/slam_processor_narrow_fov.cpp
	src/icp_scan_matcher.cpp
	src/map_filler.cpp
	src/scan_scoring.cpp
	src/map_point_cloud.cpp
	src/visualization.cpp
	src/slam_ros.cpp
)

target_link_libraries(bob_slam_lib 
	${catkin_LIBRARIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})
