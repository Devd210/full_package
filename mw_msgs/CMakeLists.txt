cmake_minimum_required(VERSION 2.8.3)
project(mw_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
)

add_message_files(FILES
  ChiefExecutiveMode.msg
  Route.msg
  Task.msg
  Duty.msg
  RobotStatus.msg
  Diagnostic.msg
  CostGrid.msg
  mission_status.msg
)

add_service_files(
  FILES
  GetPlan.srv
  GetRoute.srv
  GetTask.srv
  GetDuty.srv
  GetRouteStatus.srv
  GetTaskStatus.srv
  SetDuty.srv
  SetMode.srv
  SetRoute.srv
  SetNodeState.srv
  SetTask.srv
  SetPlan.srv
  LoadRouteFromFile.srv
  LoadTaskFromFile.srv
  LoadDutyFromFile.srv
  GetSbplPlan.srv
  SetString.srv
  
  #Service Files for costmap_2d service files

  GetDouble.srv
  GetPose.srv
  SetRobotFootprint.srv
  GetString.srv
  SetCost.srv
  GetCircumscribedRadius.srv
  ComputeCircumscribedCost.srv
)

add_action_files(DIRECTORY action
  FILES
    GetPath.action
    ExePath.action
    WaypointNavigation.action
    Recovery.action
)
generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    actionlib
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
    nav_msgs
)

# install(FILES ${PROJECT_NAME}.bmr
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

