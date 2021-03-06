cmake_minimum_required(VERSION 3.5)
project(mw_navfn_planner)
add_compile_options(-g -std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    geometry_msgs
    nav_msgs
    mw_msgs
    mw_core
    tf2_ros
    costmap_2d
    global_planner
    pluginlib
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    visualization_msgs
    geometry_msgs
    nav_msgs
    mw_msgs
    mw_core
    tf2_ros
    costmap_2d
    global_planner
    pluginlib
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/navfn_planner.cpp
  src/navfn.cpp
  src/navfn_planner_execution.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${global_planner_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES global_planner_plugin.xml custom_config.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

