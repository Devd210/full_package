cmake_minimum_required(VERSION 2.8.3)
project(chief_executive)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  mw_msgs
  roslib
  std_srvs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES novus_msgs
 CATKIN_DEPENDS std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# chief_executive
add_library(chief_executive src/chief_executive.cpp)
target_link_libraries(chief_executive ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(chief_executive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# chief_executive node
add_executable(chief_executive_node src/chief_executive_node.cpp)
add_dependencies(chief_executive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chief_executive_node chief_executive ${mathlib_LIBRARIES} ${catkin_LIBRARIES})

add_executable(set_duty_client src/set_duty_client_node.cpp)
target_link_libraries(set_duty_client yaml-cpp ${catkin_LIBRARIES})
add_dependencies(set_duty_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install( TARGETS
    chief_executive_node set_duty_client
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS
    chief_executive 
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/chief_executive/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY samples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/samples
)
