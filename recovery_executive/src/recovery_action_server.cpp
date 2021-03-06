/**
 *
 * @file recovery_server.cpp
 * @brief controller action server class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "recovery_executive/recovery_action_server.h"

namespace recovery {

RecoveryActionServer::RecoveryActionServer(const TFPtr &tf_listener_ptr)
  : private_nh_("~"),
    tf_listener_ptr_(tf_listener_ptr),
    recovery_plugin_manager_(
      "recoveries",
      boost::bind(&RecoveryActionServer::loadRecoveryPlugin, this, _1),
      boost::bind(&RecoveryActionServer::initializeRecoveryPlugin, this, _1, _2)),
    recovery_execution_plugin_manager_(
      "recovery_executions",
      boost::bind(&RecoveryActionServer::loadRecoveryExecutionPlugin, this, _1),
      boost::bind(&RecoveryActionServer::initializeRecoveryExecutionPlugin, this, _1, _2)),
    tf_timeout_(private_nh_.param<double>("tf_timeout", 3.0)),
    global_frame_(private_nh_.param<std::string>("global_frame", "map")),
    robot_frame_(private_nh_.param<std::string>("robot_frame", "base_link")),
    robot_info_(*tf_listener_ptr,
                global_frame_,
                robot_frame_,
                tf_timeout_),
    recovery_action_(new RecoveryAction(name_action_recovery, robot_info_)),
    recovery_plugin_loader_("recovery_executive",
                            "recovery::AbstractRecovery"),
    recovery_execution_plugin_loader_("recovery_executive",
                                      "recovery::AbstractRecoveryExecution") {

  ros::NodeHandle nh;

  action_server_recovery_ptr_ = ActionServerRecoveryPtr(
    new ActionServerRecovery(
      private_nh_,
      name_action_recovery,
      boost::bind(&RecoveryActionServer::callActionRecovery, this, _1),
      boost::bind(&RecoveryActionServer::cancelActionRecovery, this, _1),
      false));

  recovery_plugin_manager_.loadPlugins();
  recovery_execution_plugin_manager_.loadPlugins();

  action_server_recovery_ptr_->start();
}

RecoveryActionServer::~RecoveryActionServer() {
  for (int i = 0; i < recovery_plugin_manager_.getLoadedNames().size(); i++) {
    recovery_plugin_manager_.getPlugin(recovery_plugin_manager_.getLoadedNames()[i]).reset();
  }
  for (int i = 0; i < recovery_execution_plugin_manager_.getLoadedNames().size(); i++) {
    recovery_execution_plugin_manager_.getPlugin(recovery_execution_plugin_manager_.getLoadedNames()[i]).reset();
  }
  ROS_INFO("[Recovery Action Server]: All plugins have been successfully unloaded");
}

AbstractRecovery::Ptr RecoveryActionServer::loadRecoveryPlugin(const std::string &recovery_type) {
  AbstractRecovery::Ptr recovery_ptr;
  try {
    recovery_ptr = boost::static_pointer_cast<AbstractRecovery>(
      recovery_plugin_loader_.createInstance(recovery_type));
    std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
    ROS_DEBUG_STREAM("[Recovery Action Server]: Recovery plugin " << recovery_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("[Recovery Action Server]: Failed to load the " << recovery_type
                                                                     << " recovery, are you sure it's properly registered"
                                                                     << " and that the containing library is built? "
                                                                     << ex.what());
  }
  return recovery_ptr;
}

AbstractRecoveryExecution::Ptr RecoveryActionServer::loadRecoveryExecutionPlugin(const std::string &recovery_execution_type) {
  AbstractRecoveryExecution::Ptr recovery_execution_ptr;
  try {
    recovery_execution_ptr = boost::static_pointer_cast<AbstractRecoveryExecution>(
      recovery_execution_plugin_loader_.createInstance(recovery_execution_type));
    std::string recovery_execution_name = recovery_execution_plugin_loader_.getName(recovery_execution_type);
    ROS_DEBUG_STREAM(
      "[Recovery Action Server]: Recovery Execution plugin " << recovery_execution_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("[Recovery Action Server]: Failed to load the " << recovery_execution_type
                                                                     << " recovery execution, are you sure it's properly registered"
                                                                     << " and that the containing library is built? "
                                                                     << ex.what());
  }
  return recovery_execution_ptr;
}

bool RecoveryActionServer::initializeRecoveryPlugin(const std::string &name,
                                                    const AbstractRecovery::Ptr &recovery_ptr) {
  AbstractRecovery::Ptr
    recovery_pointer = boost::static_pointer_cast<AbstractRecovery>(recovery_ptr);
  ROS_DEBUG_STREAM("[Recovery Action Server]: Initialize recovery \"" << name << "\".");
  recovery_pointer->initialize(name, tf_listener_ptr_);
  ROS_DEBUG("[Recovery Action Server]: Recovery plugin initialized.");
  return true;
}

bool RecoveryActionServer::initializeRecoveryExecutionPlugin(const std::string &name,
                                                             const AbstractRecoveryExecution::Ptr &recovery_execution_ptr) {
  AbstractRecoveryExecution::Ptr
    recovery_execution_pointer =
    boost::static_pointer_cast<AbstractRecoveryExecution>(recovery_execution_ptr);
  ROS_DEBUG_STREAM("[Recovery Action Server]: Initialize recovery execution\"" << name << "\".");
  recovery_execution_pointer->initialize(name, recovery_plugin_, tf_listener_ptr_);
  ROS_DEBUG("[Recovery Action Server]: Recovery Execution plugin initialized.");
  return true;
}

void RecoveryActionServer::stop() {
  recovery_action_->cancelAll();
}

void RecoveryActionServer::callActionRecovery(ActionServerRecovery::GoalHandle goal_handle) {
  const mw_msgs::RecoveryGoal &goal = *(goal_handle.getGoal().get());

  std::string recovery_name;
  if (!recovery_plugin_manager_.getLoadedNames().empty()) {
    recovery_name = goal.behavior.empty() ? recovery_plugin_manager_.getLoadedNames().front() : goal.behavior;
  } else {
    mw_msgs::RecoveryResult result;
    result.outcome = mw_msgs::RecoveryResult::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    ROS_WARN("[Recovery Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
    return;
  }

  if (!recovery_plugin_manager_.hasPlugin(recovery_name)) {
    mw_msgs::RecoveryResult result;
    result.outcome = mw_msgs::RecoveryResult::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.behavior + "\"!";
    ROS_WARN("[Recovery Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
    return;
  }

  recovery_plugin_.reset();
  recovery_plugin_ = recovery_plugin_manager_.getPlugin(recovery_name);

  if (recovery_plugin_) {
    std::string recovery_execution_name;
    if (!recovery_execution_plugin_manager_.getLoadedNames().empty()) {
      recovery_execution_name =
        goal.behavior_execution.empty() ? recovery_execution_plugin_manager_.getLoadedNames().front()
                                          : goal.behavior_execution;
    } else {
      mw_msgs::RecoveryResult result;
      result.outcome = mw_msgs::RecoveryResult::INVALID_PLUGIN;
      result.message = "No plugins loaded at all!";
      ROS_WARN("[Recovery Action Server]: %s", result.message.c_str());
      goal_handle.setRejected(result, result.message);
      return;
    }

    if (!recovery_execution_plugin_manager_.hasPlugin(recovery_execution_name)) {
      mw_msgs::RecoveryResult result;
      result.outcome = mw_msgs::RecoveryResult::INVALID_PLUGIN;
      result.message = "No plugin loaded with the given name \"" + goal.behavior_execution + "\"!";
      ROS_WARN("[Recovery Action Server]: %s", result.message.c_str());
      goal_handle.setRejected(result, result.message);
      return;
    }

    recovery_execution_.reset();
    recovery_execution_ = recovery_execution_plugin_manager_.getPlugin(recovery_execution_name);
    ROS_INFO("[Recovery Action Server]: Start action \"recovery\" using recovery \"%s\" of type \"%s\"",
             recovery_name.c_str(),
             recovery_plugin_manager_.getType(recovery_name).c_str());
    if (recovery_execution_ == nullptr) {
      ROS_ERROR("[Recovery Action Server]: Could not load recovery execution plugin");
      return;
    }

    recovery_execution_->initialize(recovery_execution_name, recovery_plugin_, tf_listener_ptr_);
    // starts another recovery action
    recovery_action_->start(goal_handle, recovery_execution_);
  } else {
    mw_msgs::RecoveryResult result;
    result.outcome = mw_msgs::RecoveryResult::INTERNAL_ERROR;
    result.message = "Internal Error: \"recovery_plugin\" pointer should not be a null pointer!";
    ROS_FATAL("[Recovery Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
  }
}

void RecoveryActionServer::cancelActionRecovery(ActionServerRecovery::GoalHandle goal_handle) {
  ROS_INFO("[Recovery Action Server]: Cancel action \"recovery\"");
  recovery_action_->cancel(goal_handle);
}

} // namespace controller
