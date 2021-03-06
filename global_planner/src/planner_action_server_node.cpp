/**
 *
 * @file planner_action_server_node.cpp
 * @brief server node to serve planner action
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "global_planner/planner_action_server.h"
#include <mw_core/types.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "global_planner");

#ifdef USE_OLD_TF
  ros::NodeHandle nh("global_planner");
  TFPtr tf_listener_ptr(new TF(nh, ros::Duration(cache_time), true));
#else
  TFPtr tf_listener_ptr(new TF(ros::Duration(10)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);
#endif

  global_planner::PlannerActionServer server(tf_listener_ptr);

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return EXIT_SUCCESS;
}