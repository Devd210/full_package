/**
 *
 * @file controller_action_server.cpp
 * @brief controller action server class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "controller_executive/controller_action_server.h"

namespace controller {

ControllerActionServer::ControllerActionServer(const TFPtr &tf_listener_ptr)
    : private_nh_("~"),
      tf_listener_ptr_(tf_listener_ptr),
      controller_plugin_manager_("controllers",
                                 boost::bind(&ControllerActionServer::loadControllerPlugin,
                                             this,
                                             _1),
                                 boost::bind(&ControllerActionServer::initializeControllerPlugin,
                                             this,
                                             _1,
                                             _2)),
      controller_execution_plugin_manager_(
          "controller_executions",
          boost::bind(&ControllerActionServer::loadControllerExecutionPlugin, this, _1),
          boost::bind(&ControllerActionServer::initializeControllerExecutionPlugin, this, _1, _2)),
      tf_timeout_(private_nh_.param<double>("tf_timeout", 3.0)),
      global_frame_(private_nh_.param<std::string>("global_frame", "map")),
      robot_frame_(private_nh_.param<std::string>("robot_frame", "base_link")),
      robot_info_(*tf_listener_ptr,
                  global_frame_,
                  robot_frame_,
                  tf_timeout_),
      controller_action_(new ControllerAction(name_action_exe_path,
                                              robot_info_)),
      controller_plugin_loader_("controller_executive",
                                "controller::AbstractController"),
      controller_execution_plugin_loader_("controller_executive",
                                          "controller::AbstractControllerExecution") {

  ros::NodeHandle nh;

  action_server_exe_path_ptr_ = ActionServerExePathPtr(
      new ActionServerExePath(
          private_nh_,
          name_action_exe_path,
          boost::bind(&ControllerActionServer::callActionExePath, this, _1),
          boost::bind(&ControllerActionServer::cancelActionExePath, this, _1),
          false));

  controller_plugin_manager_.loadPlugins();
  controller_execution_plugin_manager_.loadPlugins();

  action_server_exe_path_ptr_->start();
}

ControllerActionServer::~ControllerActionServer() {
  for (int i = 0; i < controller_plugin_manager_.getLoadedNames().size(); i++) {
    controller_plugin_manager_.getPlugin(controller_plugin_manager_.getLoadedNames()[i]).reset();
  }
  for (int i = 0; i < controller_execution_plugin_manager_.getLoadedNames().size(); i++) {
    controller_execution_plugin_manager_.getPlugin(controller_execution_plugin_manager_.getLoadedNames()[i]).reset();
  }
  ROS_INFO("[Controller Action Server]: All plugins have been successfully unloaded");
}

AbstractController::Ptr ControllerActionServer::loadControllerPlugin(const std::string &controller_type) {
  controller::AbstractController::Ptr controller_ptr;
  try {
    controller_ptr = boost::static_pointer_cast<controller::AbstractController>(
        controller_plugin_loader_.createInstance(controller_type));
    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_DEBUG_STREAM("[Controller Action Server]: Controller plugin " << controller_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("[Controller Action Server]: Failed to load the " << controller_type
                                                                       << " controller, are you sure it's properly registered"
                                                                       << " and that the containing library is built? "
                                                                       << ex.what());
  }
  return controller_ptr;
}

AbstractControllerExecution::Ptr ControllerActionServer::loadControllerExecutionPlugin(const std::string &controller_execution_type) {
  controller::AbstractControllerExecution::Ptr controller_execution_ptr;
  try {
    controller_execution_ptr = boost::static_pointer_cast<controller::AbstractControllerExecution>(
        controller_execution_plugin_loader_.createInstance(controller_execution_type));
    std::string controller_execution_name = controller_execution_plugin_loader_.getName(controller_execution_type);
    ROS_DEBUG_STREAM(
        "[Controller Action Server]: Controller Execution plugin " << controller_execution_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("[Controller Action Server]: Failed to load the " << controller_execution_type
                                                                       << " controller execution, are you sure it's properly registered"
                                                                       << " and that the containing library is built? "
                                                                       << ex.what());
  }
  return controller_execution_ptr;
}

bool ControllerActionServer::initializeControllerPlugin(const std::string &name,
                                                        const controller::AbstractController::Ptr &controller_ptr) {
  controller::AbstractController::Ptr
      controller_pointer = boost::static_pointer_cast<controller::AbstractController>(controller_ptr);
  ROS_DEBUG_STREAM("[Controller Action Server]: Initialize controller \"" << name << "\".");
  controller_pointer->initialize(name, tf_listener_ptr_);
  ROS_DEBUG("[Controller Action Server]: Controller plugin initialized.");
  return true;
}

bool ControllerActionServer::initializeControllerExecutionPlugin(const std::string &name,
                                                                 const controller::AbstractControllerExecution::Ptr &controller_execution_ptr) {
  controller::AbstractControllerExecution::Ptr
      controller_execution_pointer =
      boost::static_pointer_cast<controller::AbstractControllerExecution>(controller_execution_ptr);
  ROS_DEBUG_STREAM("[Controller Action Server]: Initialize controller execution\"" << name << "\".");
  controller_execution_pointer->initialize(name, controller_plugin_, tf_listener_ptr_);
  ROS_DEBUG("[Controller Action Server]: Controller Execution plugin initialized.");
  return true;
}

void ControllerActionServer::stop() {
  controller_action_->cancelAll();
}

void ControllerActionServer::callActionExePath(ActionServerExePath::GoalHandle goal_handle) {
  const mw_msgs::ExePathGoal &goal = *(goal_handle.getGoal().get());

  std::string controller_name;
  if (!controller_plugin_manager_.getLoadedNames().empty()) {
    controller_name = goal.controller.empty() ? controller_plugin_manager_.getLoadedNames().front() : goal.controller;
  } else {
    mw_msgs::ExePathResult result;
    result.outcome = mw_msgs::ExePathResult::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    ROS_WARN("[Controller Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
    return;
  }

  if (!controller_plugin_manager_.hasPlugin(controller_name)) {
    mw_msgs::ExePathResult result;
    result.outcome = mw_msgs::ExePathResult::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.controller + "\"!";
    ROS_WARN("[Controller Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
    return;
  }

  controller_plugin_.reset();
  controller_plugin_ = controller_plugin_manager_.getPlugin(controller_name);
  ROS_INFO("[Controller Action Server]: Start action \"exe_path\" using controller \"%s\" of type \"%s\"",
           controller_name.c_str(),
           controller_plugin_manager_.getType(controller_name).c_str());

  if (controller_plugin_) {
    std::string controller_execution_name;
    if (!controller_execution_plugin_manager_.getLoadedNames().empty()) {
      controller_execution_name =
          goal.controller_execution.empty() ? controller_execution_plugin_manager_.getLoadedNames().front()
                                            : goal.controller_execution;
    } else {
      mw_msgs::ExePathResult result;
      result.outcome = mw_msgs::ExePathResult::INVALID_PLUGIN;
      result.message = "No plugins loaded at all!";
      ROS_WARN("[Controller Action Server]: %s", result.message.c_str());
      goal_handle.setRejected(result, result.message);
      return;
    }

    if (!controller_execution_plugin_manager_.hasPlugin(controller_execution_name)) {
      mw_msgs::ExePathResult result;
      result.outcome = mw_msgs::ExePathResult::INVALID_PLUGIN;
      result.message = "No plugin loaded with the given name \"" + goal.controller_execution + "\"!";
      ROS_WARN("[Controller Action Server]: %s", result.message.c_str());
      goal_handle.setRejected(result, result.message);
      return;
    }

    controller_execution_.reset();
    controller_execution_ = controller_execution_plugin_manager_.getPlugin(controller_execution_name);
    ROS_INFO("[Controller Action Server]: Start action \"exe_path\" using controller execution\"%s\" of type \"%s\"",
             controller_execution_name.c_str(),
             controller_execution_plugin_manager_.getType(controller_execution_name).c_str());
    if (controller_execution_ == nullptr) {
      ROS_ERROR("[Controller Action Server]: Could not load controller execution plugin");
      return;
    }

    controller_execution_->initialize(controller_execution_name, controller_plugin_, tf_listener_ptr_);
    // starts another controller action
    controller_action_->start(goal_handle, controller_execution_);
  } else {
    mw_msgs::ExePathResult result;
    result.outcome = mw_msgs::ExePathResult::INTERNAL_ERROR;
    result.message = "Internal Error: \"controller_plugin\" pointer should not be a null pointer!";
    ROS_FATAL("[Controller Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
  }
}

void ControllerActionServer::cancelActionExePath(ActionServerExePath::GoalHandle goal_handle) {
  ROS_INFO("[Controller Action Server]: Cancel action \"exe_path\"");
  controller_action_->cancel(goal_handle);
}

} // namespace controller