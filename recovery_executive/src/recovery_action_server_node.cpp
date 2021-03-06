/**
 *
 * @file recovery_action_node.cpp
 * @brief server node to serve recovery action
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/recovery_action_server.h"
#include <mw_core/types.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "recovery_executive");

#ifdef USE_OLD_TF
  ros::NodeHandle nh("controller_executive");
  TFPtr tf_listener_ptr(new TF(nh, ros::Duration(cache_time), true));
#else
  TFPtr tf_listener_ptr(new TF(ros::Duration(10)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);
#endif

  recovery::RecoveryActionServer server(tf_listener_ptr);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
