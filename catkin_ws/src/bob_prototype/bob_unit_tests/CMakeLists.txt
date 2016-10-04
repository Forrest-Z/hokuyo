cmake_minimum_required(VERSION 2.8.3)
project(bob_unit_tests)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
bob_coverage
bob_toolbox
bob_grid_map
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES bob_unit_tests
  CATKIN_DEPENDS 
bob_coverage
bob_toolbox
bob_grid_map
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_library(bob_unit_tests
#  src/<some_file>.cpp
#  src/<other_file>.cpp
#)

#target_link_libraries(bob_unit_tests ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-test 
  src/main.cpp

  bob_coverage/area_test.cpp

  bob_toolbox/world_point_test.cpp
  bob_toolbox/angular_test.cpp
  bob_toolbox/line_geometry_test.cpp
  bob_toolbox/matrix_test.cpp

  bob_grid_map/map_resize.cpp
)

if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
