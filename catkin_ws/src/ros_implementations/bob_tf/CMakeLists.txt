cmake_minimum_required(VERSION 2.8)
project(bob_tf)

find_package(catkin REQUIRED)

find_package(Boost COMPONENTS thread signals system REQUIRED)
find_package(catkin COMPONENTS angles geometry_msgs message_filters message_generation tf tf2 rosconsole roscpp rostime sensor_msgs std_msgs REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR}
                           ${catkin_INCLUDE_DIRS}
)
include_directories(include)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(static_transform_publisher src/static_transform_publisher.cpp)
target_link_libraries(static_transform_publisher ${PROJECT_NAME})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_filters message_runtime roscpp sensor_msgs std_msgs rosconsole
)

add_library(${PROJECT_NAME} src/transform_listener.cpp src/transform_broadcaster.cpp src/cache.cpp src/buffer_core.cpp src/static_cache.cpp src/buffer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp tf2_msgs_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}  static_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

