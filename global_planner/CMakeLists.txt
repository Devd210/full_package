cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-g -std=c++11)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED
  COMPONENTS
  std_msgs
  geometry_msgs
  mw_core
  pluginlib
  dynamic_reconfigure
  mw_msgs
  message_generation
  roscpp
  tf2_ros
  nav_msgs
  costmap_2d
)

generate_dynamic_reconfigure_options(
  cfg/SimplePlanner.cfg
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  tf
  mw_core
  pluginlib
  dynamic_reconfigure
  mw_msgs
  message_generation
  roscpp
  tf2_ros
  nav_msgs
  costmap_2d
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/planner_action.cpp
  src/base_planner_execution.cpp
  src/simple_planner.cpp
  src/simple_planner_execution.cpp
  src/planner_action_server.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(global_planner_action_node src/planner_action_server_node.cpp)
add_dependencies(global_planner_action_node mw_msgs_generate_messages_cpp)
target_link_libraries(global_planner_action_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_planner_plugin unit_test/test_planner_plugin)
target_link_libraries(test_planner_plugin ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS
  global_planner_action_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES global_planner_plugins.xml custom_config.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
