cmake_minimum_required(VERSION 2.8.3)
project(executive)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-g -std=c++11)
cmake_policy(SET CMP0054 NEW)

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
  mw_core
  diagnostic
  roslib
  std_srvs
  mlicense
  tf2
  xmlrpcpp
)

#find_library(DIAGNOSTIC liblocal_diagnostic)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs geometry_msgs std_srvs xmlrpcpp
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mission_executive.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#mission_executive_node
add_executable(mission_executive_node src/mission_executive_node.cpp)
add_dependencies(mission_executive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_executive_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${mathlib_LIBRARIES})

# executive_node
add_executable(set_route_client src/set_route_client_node.cpp)
target_link_libraries(set_route_client ${${PROJECT_NAME}_LIBRARIES} yaml-cpp ${catkin_LIBRARIES})
add_dependencies(set_route_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_plan_client src/set_plan_client_node.cpp)
target_link_libraries(set_plan_client ${catkin_LIBRARIES})
add_dependencies(set_plan_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(costmap_plan_test unit_tests/costmap_plan_test.cpp)
target_link_libraries(costmap_plan_test ${catkin_LIBRARIES})
add_dependencies(costmap_plan_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simple_plan_test unit_tests/simple_plan_test.cpp)
target_link_libraries(simple_plan_test ${catkin_LIBRARIES})
add_dependencies(simple_plan_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  mission_executive_node set_plan_client set_route_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  executive
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/executive/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
