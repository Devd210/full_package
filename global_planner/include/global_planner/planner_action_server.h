/**
 * @file planner_action_server.h
 * @brief Contains the node that launches the global planner action
 * @author Sourav Agrawal<sourav.agrawal@mowito.in>
 */

#ifndef GLOBAL_PLANNER_PLANNER_ACTION_SERVER_H
#define GLOBAL_PLANNER_PLANNER_ACTION_SERVER_H

#include <actionlib/server/action_server.h>
#include <pluginlib/class_loader.h>
#include <mw_msgs/GetPathAction.h>

#include <global_planner/base_planner_execution.h>
#include <global_planner/planner_action.h>

#include <mw_core/robot_information.h>
#include <mw_core/abstract_classes/abstract_plugin_manager.h>

namespace global_planner {

typedef actionlib::ActionServer<mw_msgs::GetPathAction> ActionServerGetPath;
typedef boost::shared_ptr<ActionServerGetPath> ActionServerGetPathPtr;

std::string name_action_get_path = "get_path";

/**
 * @class PlannerActionServer
 * @brief The class for the global planner action node
 */
class PlannerActionServer {

 private:
  ros::NodeHandle private_nh_;

  mw_core::AbstractPluginManager<global_planner::AbstractPlanner> planner_plugin_manager_;
  mw_core::AbstractPluginManager<global_planner::AbstractPlannerExecution> planner_execution_plugin_manager_;

  ActionServerGetPathPtr action_server_get_path_ptr_;

  std::string robot_frame_;
  std::string global_frame_;

  ros::Duration tf_timeout_;

  const TFPtr tf_listener_ptr_;

  mw_core::RobotInformation robot_info_;

  PlannerAction::Ptr planner_action_;

  pluginlib::ClassLoader<global_planner::AbstractPlanner> planner_plugin_loader_;
  pluginlib::ClassLoader<global_planner::AbstractPlannerExecution> planner_execution_plugin_loader_;

  global_planner::AbstractPlanner::Ptr planner_plugin_;
  global_planner::AbstractPlannerExecution::Ptr planner_execution_plugin_;

 public:
  explicit PlannerActionServer(const TFPtr &tf_listener_ptr);
  ~PlannerActionServer();

  // Loads all the specified planner plugins
  global_planner::AbstractPlanner::Ptr loadPlannerPlugin(const std::string &planner_type);

  // Loads all the specified planner execution plugins
  global_planner::AbstractPlannerExecution::Ptr loadPlannerExecutionPlugin(const std::string &planner_execution_type);

  // Inits the planner plugin
  bool initializePlannerPlugin(
      const std::string &name,
      const global_planner::AbstractPlanner::Ptr &planner_ptr);

  // Inits the planner execution plugin
  bool initializePlannerExecutionPlugin(
      const std::string &name,
      const global_planner::AbstractPlannerExecution::Ptr &planner_execution_ptr);

  // Called when a new goal is received
  void callActionGetPath(ActionServerGetPath::GoalHandle goal_handle);

  // Called when the cancel request is received
  void cancelActionGetPath(ActionServerGetPath::GoalHandle goal_handle);

  // Used to stop all the current goals
  void stop();
};

}

#endif //GLOBAL_PLANNER_PLANNER_ACTION_SERVER_H
