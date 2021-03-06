cmake_minimum_required(VERSION 2.8.3)
project(task_executive)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
  nav_msgs
  mw_msgs
  diagnostic
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs
  #  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(task_executive src/task_executive.cpp)
target_link_libraries(task_executive ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(task_executive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_executive_node src/task_executive_node.cpp)
add_dependencies(task_executive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_executive_node task_executive ${mathlib_LIBRARIES} ${catkin_LIBRARIES})

add_executable(set_task_client src/set_task_client_node.cpp)
target_link_libraries(set_task_client yaml-cpp ${catkin_LIBRARIES})
add_dependencies(set_task_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install( TARGETS
    task_executive_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS
    task_executive
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/task_executive/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
