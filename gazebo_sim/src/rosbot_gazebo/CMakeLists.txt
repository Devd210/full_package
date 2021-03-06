cmake_minimum_required(VERSION 2.8.3)
project(rosbot_gazebo)

cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	gazebo_dev
	geometry_msgs
	nav_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

link_directories(
	${catkin_LIBRARY_DIRS}
)

catkin_package(
	INCLUDE_DIRS
	LIBRARIES
		gazebo_ros_skid_steer_drive_with_noise
	CATKIN_DEPENDS
		roscpp
		geometry_msgs
		nav_msgs
)

add_library(gazebo_ros_skid_steer_drive_with_noise plugins/gazebo_ros_skid_steer_drive.cpp)
target_link_libraries(gazebo_ros_skid_steer_drive_with_noise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS gazebo_ros_skid_steer_drive_with_noise
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
