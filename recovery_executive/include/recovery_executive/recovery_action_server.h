/**
 *
 * @file recovery_action_server.h
 * @brief controller action server class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_RECOVERY_ACTION_SERVER_H
#define RECOVERY_EXECUTIVE_RECOVERY_ACTION_SERVER_H

#include <ros/ros.h>

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <pluginlib/class_loader.h>

#include <actionlib/server/action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mw_msgs/RecoveryAction.h>

#include <mw_core/types.h>
#include <mw_core/utility_functions.h>
#include <mw_core/robot_information.h>

#include <mw_core/abstract_classes/abstract_plugin_manager.h>

#include <recovery_executive/base_recovery.h>
#include <recovery_executive/base_recovery_execution.h>
#include <recovery_executive/recovery_action.h>

namespace recovery {

typedef actionlib::ActionServer<mw_msgs::RecoveryAction> ActionServerRecovery;
typedef boost::shared_ptr<ActionServerRecovery> ActionServerRecoveryPtr;

const std::string name_action_recovery = "recovery";

class RecoveryActionServer {

public:

  explicit RecoveryActionServer(const TFPtr &tf_listener_ptr);

  ~RecoveryActionServer();

  void stop();

  AbstractRecovery::Ptr loadRecoveryPlugin(const std::string &recovery_type);

  AbstractRecoveryExecution::Ptr loadRecoveryExecutionPlugin(const std::string &recovery_execution_type);

  bool initializeRecoveryPlugin(const std::string &name,
                                const AbstractRecovery::Ptr &recovery_ptr);

  bool initializeRecoveryExecutionPlugin(const std::string &name,
                                         const AbstractRecoveryExecution::Ptr &recovery_execution_ptr);

  void callActionRecovery(ActionServerRecovery::GoalHandle goal_handle);

  void cancelActionRecovery(ActionServerRecovery::GoalHandle goal_handle);

private:

  ros::NodeHandle private_nh_;

  mw_core::AbstractPluginManager<AbstractRecovery> recovery_plugin_manager_;
  mw_core::AbstractPluginManager<AbstractRecoveryExecution> recovery_execution_plugin_manager_;

  ActionServerRecoveryPtr action_server_recovery_ptr_;

  std::string robot_frame_;

  //! the global frame, in which the robot is moving
  std::string global_frame_;

  //! timeout after tf returns without a result
  ros::Duration tf_timeout_;

  //! shared pointer to the common TransformListener
  const TFPtr tf_listener_ptr_;

  mw_core::RobotInformation robot_info_;

  RecoveryAction::Ptr recovery_action_;

  pluginlib::ClassLoader<AbstractRecovery> recovery_plugin_loader_;
  pluginlib::ClassLoader<AbstractRecoveryExecution> recovery_execution_plugin_loader_;

  AbstractRecovery::Ptr recovery_plugin_;
  AbstractRecoveryExecution::Ptr recovery_execution_;
};

}  // namespace recovery

#endif  // RECOVERY_EXECUTIVE_RECOVERY_ACTION_SERVER_H
