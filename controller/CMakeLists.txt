cmake_minimum_required(VERSION 2.8.3)

project(controller_executive)

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
    dynamic_reconfigure
    pluginlib
)

find_package(Boost REQUIRED
  COMPONENTS
    system
    thread
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/TrajectoryPlanner.cfg
  cfg/PurePursuitController.cfg
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
    dynamic_reconfigure
    pluginlib
  LIBRARIES
    ${PROJECT_NAME}
    controller_plugins
    trajectory_planner
)

check_include_file(sys/time.h HAVE_SYS_TIME_H)
if (HAVE_SYS_TIME_H)
  add_definitions(-DHAVE_SYS_TIME_H)
endif (HAVE_SYS_TIME_H)

add_library(${PROJECT_NAME}
  src/base_controller_execution.cpp
  src/controller_action.cpp
  src/controller_action_server.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(trajectory_planner
  src/trajectory_planner/costmap_model.cpp
  src/trajectory_planner/footprint_helper.cpp
  src/trajectory_planner/goal_functions.cpp
  src/trajectory_planner/map_cell.cpp
  src/trajectory_planner/map_grid.cpp
  src/trajectory_planner/map_grid_visualizer.cpp
  src/trajectory_planner/trajectory.cpp
  src/trajectory_planner/trajectory_planner.cpp
)
add_dependencies(trajectory_planner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES})

add_library(controller_plugins
  src/pure_pursuit_controller/pure_pursuit_controller.cpp
  src/pure_pursuit_controller/pure_pursuit_controller_execution.cpp
  src/trajectory_planner/trajectory_planner_ros.cpp
  src/trajectory_planner/trajectory_planner_ros_execution.cpp
)
add_dependencies(controller_plugins ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(controller_plugins trajectory_planner ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(controller_action_node src/controller_action_server_node.cpp)
add_dependencies(controller_action_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(controller_action_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS
  controller_action_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  controller_plugins
  trajectory_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugins.xml custom_config.conf
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
