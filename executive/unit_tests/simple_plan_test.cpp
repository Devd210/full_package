#include <mw_msgs/GetPathAction.h>
#include <mw_msgs/ExePathAction.h>
#include <ros/ros.h>
#include <mw_core/data_types/geometry_types.h>
#include <tf2/utils.h>
#include <mw_core/data_types/geometry_types.h>
#include <mw_core/utility_functions.h>
#include <actionlib/client/simple_action_client.h>

using mw_core::Point;

ros::Publisher plan_pub;
ros::Publisher cmd_vel_pub;
bool running = true;
nav_msgs::Path plan;

void donePlannerCB(const actionlib::SimpleClientGoalState &state, const mw_msgs::GetPathResultConstPtr result) {
  ROS_INFO("Action has finished with status: %s", result->message.c_str());
  if (result->path.poses.size() > 0) {
    ROS_INFO("The cost of the computed path is %lf", result->cost);
    plan = result->path;
    plan_pub.publish(result->path);
  }
  //ROS_DEBUG("Shutting down now!");
  running = false;
  ROS_DEBUG("Lets go baby!");
  //exit(0);
}

void activePlannerCB() {
  ROS_INFO("Plan requested,lets see how it goes");
}

void feedbackPlannerCB(const mw_msgs::GetPathFeedbackConstPtr feedback) {
  ROS_INFO("Planner is planning!");
}

void doneControllerCB(const actionlib::SimpleClientGoalState &state, const mw_msgs::ExePathResultConstPtr &result) {
  ROS_INFO("The controller has finished with status %s", state.toString().c_str());
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;
  cmd_vel_pub.publish(cmd_vel);
  ROS_INFO("Published an explicit zero velocity");
  running = false;
}

void activeControllerCB() {
  ROS_INFO("Controller has received the global plan, let's see how it goes!");
}

void feedbackControllerCB(const mw_msgs::ExePathFeedbackConstPtr &feedback) {
  ROS_DEBUG("Received feedback from controller with message: %s", feedback->message.c_str());
  cmd_vel_pub.publish(feedback->last_cmd_vel.twist);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simple_planner_test_node");
  ros::NodeHandle nh("~");
  plan_pub = nh.advertise<nav_msgs::Path>("/path", 2);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  actionlib::SimpleActionClient<mw_msgs::GetPathAction>
      factory_planner_action("/global_planner/get_path", true);
  factory_planner_action.waitForServer();
  ROS_INFO("Planner Action Server is up and running");
  mw_msgs::GetPathGoal goal;
  std::vector<mw_core::Point> waypoints_pts;
  std::vector<geometry_msgs::PoseStamped> waypoints_stamped;

  waypoints_pts.emplace_back(Point(0.0, 0.0, 0.0));
  waypoints_pts.emplace_back(Point(1.0, 0.0, 0.0));
  waypoints_pts.emplace_back(Point(1.0, 2.0, 0.0));
  waypoints_pts.emplace_back(Point(0.0, 2.0, 0.0));

  mw_core::convertPointToPose(waypoints_pts, waypoints_stamped);

  goal.waypoints = waypoints_stamped;
  goal.planner = "genroute";
  goal.concurrency_slot = 0;
  //ROS_INFO("Sleep Started");
  ros::Duration(1.5).sleep();
  //ROS_INFO("Sleep Completed");
  //pub.publish(goal);
  factory_planner_action.sendGoal(goal, &donePlannerCB, &activePlannerCB, &feedbackPlannerCB);

  while (ros::ok() && running)
    ros::spinOnce();

  actionlib::SimpleActionClient<mw_msgs::ExePathAction> controller("/controller_executive/exe_path", true);
  controller.waitForServer();
  ROS_INFO("Controller is up and running");
  mw_msgs::ExePathGoal exe_path_goal;
  exe_path_goal.concurrency_slot = 0;
  exe_path_goal.controller = "trajectory_planner";
  exe_path_goal.controller_execution = "trajectory_planner_execution";
  exe_path_goal.path = plan;

  running = true;
  controller.sendGoal(exe_path_goal, &doneControllerCB, &activeControllerCB, &feedbackControllerCB);
  controller.waitForResult();

  return 0;
}