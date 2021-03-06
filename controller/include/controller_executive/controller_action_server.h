/**
 *
 * @file controller_action_server.h
 * @brief controller action server class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef CONTROLLER_EXECUTIVE_CONTROLLER_ACTION_H
#define CONTROLLER_EXECUTIVE_CONTROLLER_ACTION_H

#include <ros/ros.h>

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <pluginlib/class_loader.h>

#include <actionlib/server/action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <mw_core/types.h>
#include <mw_core/utility_functions.h>
#include <mw_core/robot_information.h>

#include <mw_core/abstract_classes/abstract_plugin_manager.h>

#include <mw_msgs/ExePathAction.h>

#include <controller_executive/controller_action.h>
#include <controller_executive/base_controller.h>
#include <controller_executive/base_controller_execution.h>

namespace controller {

typedef actionlib::ActionServer<mw_msgs::ExePathAction> ActionServerExePath;
typedef boost::shared_ptr<ActionServerExePath> ActionServerExePathPtr;

const std::string name_action_exe_path = "exe_path";

class ControllerActionServer {

 public:

  explicit ControllerActionServer(const TFPtr &tf_listener_ptr);

  ~ControllerActionServer();

  void stop();

  AbstractController::Ptr loadControllerPlugin(const std::string &controller_type);

  AbstractControllerExecution::Ptr loadControllerExecutionPlugin(const std::string &controller_execution_type);

  bool initializeControllerPlugin(const std::string &name,
                                  const AbstractController::Ptr &controller_ptr);

  bool initializeControllerExecutionPlugin(const std::string &name,
                                           const AbstractControllerExecution::Ptr &controller_execution_ptr);

  void callActionExePath(ActionServerExePath::GoalHandle goal_handle);

  void cancelActionExePath(ActionServerExePath::GoalHandle goal_handle);

 private:

  ros::NodeHandle private_nh_;

  mw_core::AbstractPluginManager<controller::AbstractController> controller_plugin_manager_;
  mw_core::AbstractPluginManager<controller::AbstractControllerExecution> controller_execution_plugin_manager_;

  ActionServerExePathPtr action_server_exe_path_ptr_;

  std::string robot_frame_;

  //! the global frame, in which the robot is moving
  std::string global_frame_;

  //! timeout after tf returns without a result
  ros::Duration tf_timeout_;

  //! shared pointer to the common TransformListener
  const TFPtr tf_listener_ptr_;

  //! current robot pose; moving controller is responsible to update it by calling getRobotPose
  geometry_msgs::PoseStamped robot_pose_;

  //! current goal pose; used to compute remaining distance and angle
  geometry_msgs::PoseStamped goal_pose_;

  bool clearing_rotation_allowed_;

  mw_core::RobotInformation robot_info_;

  ControllerAction::Ptr controller_action_;

  pluginlib::ClassLoader<controller::AbstractController> controller_plugin_loader_;
  pluginlib::ClassLoader<controller::AbstractControllerExecution> controller_execution_plugin_loader_;

  controller::AbstractController::Ptr controller_plugin_;
  controller::AbstractControllerExecution::Ptr controller_execution_;

};

}

#endif //CONTROLLER_EXECUTIVE_CONTROLLER_ACTION_H
