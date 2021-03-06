cmake_minimum_required(VERSION 2.8.3)
project(mw_teb)

# Set to Release in order to speed up the program significantly
#set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    #geometry_msgs
    #nav_msgs
    controller_executive
    mw_core
    mw_msgs
    costmap_2d
    dynamic_reconfigure
    pluginlib
    #base_local_planner
    #costmap_converter
    cmake_modules
    interactive_markers
    #message_generation
    nav_msgs
    mbf_costmap_core  
    mbf_msgs
    #tf2
    tf2_eigen
)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## C++11 support
## Unfortunately, the 3d-party dependency libg2o requires c++11 starting from ROS Jade.
## Even if the ROS Jade specifications do not want c++11-only packages,
## we cannot compile without c++11 enabled. Another option would be to downgrade  
## libg2o third-party package.
## By now, if you do not want c++11, please refer to the ros indigo version.
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required 
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
endif()
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/TebLocalPlannerReconfigure.cfg
)

catkin_package(
  INCLUDE_DIRS  include ${EXTERNAL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    controller_executive
    roscpp
    #geometry_msgs
    nav_msgs
    mw_core
    mw_msgs
    costmap_2d
	  costmap_converter
    dynamic_reconfigure
    pluginlib
    #base_local_planner
    interactive_markers
    #message_runtime
    #nav_core
    #nav_msgs
    #tf2
    #tf2_ros
    visualization_msgs
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS SUITESPARSE G2O
)
include_directories(include)
include_directories(
  SYSTEM
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_library(mw_teb
   src/mw_teb/timed_elastic_band.cpp
   src/mw_teb/optimal_planner.cpp
   src/mw_teb/obstacles.cpp
   src/mw_teb/visualization.cpp
   src/mw_teb/recovery_behaviors.cpp
   src/mw_teb/teb_config.cpp
   src/mw_teb/homotopy_class_planner.cpp
   src/mw_teb/teb_local_planner_ros.cpp
   src/mw_teb/graph_search.cpp
   src/mw_teb/costmap_model.cpp
   src/mw_teb/goal_functions.cpp
   src/mw_teb/teb_local_planner_ros.cpp
   src/mw_teb/teb_local_planner_ros_execution.cpp
)
# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(mw_teb ${PROJECT_NAME}_gencfg)

target_link_libraries(mw_teb
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)

#add_executable(test_optim_node src/test_optim_node.cpp)

target_link_libraries(#test_optim_node
   mw_teb
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)


install(PROGRAMS
  scripts/cmd_vel_to_ackermann_drive.py
  scripts/export_to_mat.py
  scripts/export_to_svg.py
  scripts/publish_dynamic_obstacle.py
  scripts/publish_test_obstacles.py
  scripts/publish_viapoints.py
  scripts/visualize_velocity_profile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS mw_teb
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
#install(TARGETS test_optim_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  controller_plugins.xml custom_config.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  cfg scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
