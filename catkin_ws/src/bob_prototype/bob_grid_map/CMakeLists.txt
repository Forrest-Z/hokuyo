cmake_minimum_required(VERSION 2.8.3)
project(bob_grid_map)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
	bob_toolbox
	bob_config
	bob_system
        )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)


catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES bob_grid_map 
    CATKIN_DEPENDS
	bob_toolbox
	bob_config
	bob_system
    DEPENDS
        Boost
)

add_library(bob_grid_map
  src/costmap.cpp
  src/concrete_inflated_map.cpp
  src/map_location_set.cpp
  src/concrete_map.cpp
  src/counting_probability_cell.cpp
  src/decay_probability_cell.cpp
  src/location_mapper.cpp
  src/toolbox.cpp
  src/distance_cache_matrix.cpp
)

target_link_libraries(bob_grid_map
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
