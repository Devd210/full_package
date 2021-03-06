cmake_minimum_required(VERSION 3.6)
project(mlicense)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenSSL 1.0.2 REQUIRED)

add_library(jwt-cpp INTERFACE)
add_library(jwt-cpp::jwt-cpp ALIAS jwt-cpp)

target_link_libraries(jwt-cpp INTERFACE OpenSSL::SSL OpenSSL::Crypto)

set(ENV{BUILD_CPR_TESTS} OFF)
set(ENV{USE_SYSTEM_GTEST} ON)
set(ENV{USE_SYSTEM_CURL} ON)
add_subdirectory(cpr)

catkin_package(
        DEPENDS
        INCLUDE_DIRS include ${CPR_INCLUDE_DIRS}
        LIBRARIES mlicense
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CPR_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/license.cpp
        )


add_dependencies(mlicense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mlicense
        ${catkin_LIBRARIES}
        jwt-cpp
        ${CPR_LIBRARIES}
        )

add_executable(test_mlicense
        src/test_license.cpp
        )
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(mlicense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mlicense
        ${catkin_LIBRARIES}
        jwt-cpp
        ${CPR_LIBRARIES}
        )

add_dependencies(test_mlicense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_mlicense mlicense ${catkin_LIBRARIES})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination

catkin_install_python(PROGRAMS
   scripts/robot_reg.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

# install targets
install(TARGETS mlicense
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(TARGETS test_mlicense
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation, individual folder outside the project
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        ) 

install(DIRECTORY ${CPR_INCLUDE_DIRS}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
