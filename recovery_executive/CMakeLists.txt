cmake_minimum_required(VERSION 2.8.3)

project(recovery_executive)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  mw_core
  mw_msgs
  costmap_2d
  controller_executive
  dynamic_reconfigure
  pluginlib
)

find_package(Boost REQUIRED
  COMPONENTS
  system
  thread
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    mw_core
    mw_msgs
    costmap_2d
    controller_executive
    dynamic_reconfigure
    pluginlib
  LIBRARIES
    ${PROJECT_NAME}
    recovery_plugins
)

add_library(${PROJECT_NAME}
  src/base_recovery_execution.cpp
  src/recovery_action.cpp
  src/recovery_action_server.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(recovery_plugins
  plugins/rotate_recovery.cpp
  plugins/rotate_recovery_execution.cpp
  plugins/clear_costmap_recovery.cpp
  plugins/clear_costmap_recovery_execution.cpp
  plugins/stepback_and_steerturn_recovery.cpp
  plugins/stepback_and_steerturn_recovery_execution.cpp
  plugins/back_up_recovery.cpp
  plugins/back_up_recovery_execution.cpp
)
add_dependencies(recovery_plugins ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(recovery_plugins ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(recovery_action_node src/recovery_action_server_node.cpp)
add_dependencies(recovery_action_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(recovery_action_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS
  recovery_action_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  recovery_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES recovery_plugins.xml custom_config.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
