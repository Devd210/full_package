/**
 * @file planner_action_server.cpp
 * @brief Contains the node that launches the global planner action
 * @author Sourav Agrawal<sourav.agrawal@mowito.in>
 */

#include "global_planner/planner_action_server.h"

namespace global_planner {

PlannerActionServer::PlannerActionServer(const TFPtr &tf_listener_ptr)
    : tf_listener_ptr_(tf_listener_ptr), private_nh_("~"),
      planner_plugin_manager_("planners",
                              boost::bind(&PlannerActionServer::loadPlannerPlugin, this, _1),
                              boost::bind(&PlannerActionServer::initializePlannerPlugin, this, _1, _2)),
      planner_execution_plugin_manager_("planner_executions",
                                        boost::bind(&PlannerActionServer::loadPlannerExecutionPlugin, this, _1),
                                        boost::bind(&PlannerActionServer::initializePlannerExecutionPlugin,
                                                    this, _1, _2)),
      tf_timeout_(private_nh_.param<double>("tf_timeout", 3.0)),
      global_frame_(private_nh_.param<std::string>("global_frame", "map")),
      robot_frame_(private_nh_.param<std::string>("robot_frame", "base_link")),
      robot_info_(*tf_listener_ptr_, global_frame_, robot_frame_, tf_timeout_),
      planner_plugin_loader_("global_planner", "global_planner::AbstractPlanner"),
      planner_execution_plugin_loader_("global_planner", "global_planner::AbstractPlannerExecution") {

  ros::NodeHandle nh;

  name_action_get_path = "get_path";

  planner_action_.reset(new PlannerAction(name_action_get_path, robot_info_));

  action_server_get_path_ptr_ = ActionServerGetPathPtr(
      new ActionServerGetPath(
          private_nh_,
          name_action_get_path,
          boost::bind(&PlannerActionServer::callActionGetPath, this, _1),
          boost::bind(&PlannerActionServer::cancelActionGetPath, this, _1),
          false));

  planner_plugin_manager_.loadPlugins();
  planner_execution_plugin_manager_.loadPlugins();

  action_server_get_path_ptr_->start();
}

PlannerActionServer::~PlannerActionServer() {
  for (int i = 0; i < planner_plugin_manager_.getLoadedNames().size(); i++) {
    planner_plugin_manager_.getPlugin(planner_plugin_manager_.getLoadedNames()[i]).reset();
  }
  for (int i = 0; i < planner_execution_plugin_manager_.getLoadedNames().size(); i++) {
    planner_execution_plugin_manager_.getPlugin(planner_execution_plugin_manager_.getLoadedNames()[i]).reset();
  }
  ROS_INFO("[Planner Action Server]: All plugins have been successfully unloaded");
}

void PlannerActionServer::callActionGetPath(ActionServerGetPath::GoalHandle goal_handle) {
  const mw_msgs::GetPathGoal &goal = *(goal_handle.getGoal().get());

  std::string planner_name;
  if (!planner_plugin_manager_.getLoadedNames().empty()) {
    planner_name = goal.planner.empty() ? planner_plugin_manager_.getLoadedNames().front() : goal.planner;
  } else {
    mw_msgs::GetPathResult result;
    result.outcome = mw_msgs::GetPathResult::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    ROS_WARN_STREAM("[Planner Action Server]: " << result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  if (!planner_plugin_manager_.hasPlugin(planner_name)) {
    mw_msgs::GetPathResult result;
    result.outcome = mw_msgs::GetPathResult::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.planner + "\"!";
    ROS_WARN_STREAM("[Planner Action Server]: " << result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  planner_plugin_.reset();
  planner_plugin_ = planner_plugin_manager_.getPlugin(planner_name);
  ROS_INFO_STREAM("[Planner Action Server]: Start action \"get_path\" using planner \"" << planner_name
                                                                                        << "\" of type \""
                                                                                        << planner_plugin_manager_.getType(
                                                                                            planner_name)
                                                                                        << "\"");

  if (planner_plugin_) {
    std::string planner_execution_name;
    if (!planner_execution_plugin_manager_.getLoadedNames().empty()) {
      planner_execution_name =
          goal.planner_execution.empty() ? planner_execution_plugin_manager_.getLoadedNames().front()
                                         : goal.planner_execution;
    } else {
      mw_msgs::GetPathResult result;
      result.outcome = mw_msgs::GetPathResult::INVALID_PLUGIN;
      result.message = "No plugins loaded at all!";
      ROS_WARN("[Planner Action Server]: %s", result.message.c_str());
      goal_handle.setRejected(result, result.message);
      return;
    }

    if (!planner_execution_plugin_manager_.hasPlugin(planner_execution_name)) {
      mw_msgs::GetPathResult result;
      result.outcome = mw_msgs::GetPathResult::INVALID_PLUGIN;
      result.message = "No plugin loaded with the given name \"" + goal.planner_execution + "\"!";
      ROS_WARN("[Planner Action Server]: %s", result.message.c_str());
      goal_handle.setRejected(result, result.message);
      return;
    }

    planner_execution_plugin_.reset();
    planner_execution_plugin_ = planner_execution_plugin_manager_.getPlugin(planner_execution_name);
    ROS_INFO("[Planner Action Server]: Start action \"get_path\" using planner execution\"%s\" of type \"%s\"",
             planner_execution_name.c_str(),
             planner_execution_plugin_manager_.getType(planner_execution_name).c_str());
    if (planner_execution_plugin_ == nullptr) {
      ROS_ERROR("[Planner Action Server]: Could not load planner execution plugin");
      return;
    }

    planner_execution_plugin_->initialize(planner_execution_name, planner_plugin_);
    // starts another planner action
    planner_action_->start(goal_handle, planner_execution_plugin_);
  } else {
    mw_msgs::GetPathResult result;
    result.outcome = mw_msgs::GetPathResult::INTERNAL_ERROR;
    result.message = "Internal Error: \"planner_plugin\" pointer should not be a null pointer!";
    ROS_FATAL("[Planner Action Server]: %s", result.message.c_str());
    goal_handle.setRejected(result, result.message);
  }
}

void PlannerActionServer::cancelActionGetPath(ActionServerGetPath::GoalHandle goal_handle) {
  ROS_INFO_STREAM("[Planner Action Server]: Cancel action \"get_path\"");
  planner_action_->cancel(goal_handle);
}

void PlannerActionServer::stop() {
  planner_action_->cancelAll();
}

global_planner::AbstractPlanner::Ptr PlannerActionServer::loadPlannerPlugin(const std::string &planner_type) {
  global_planner::AbstractPlanner::Ptr planner_ptr;
  try {
    planner_ptr = boost::static_pointer_cast<global_planner::AbstractPlanner>(
        planner_plugin_loader_.createInstance(planner_type));
    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_DEBUG_STREAM(
        "[Planner Action Server]: global_planner-based planner plugin " << planner_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("[Planner Action Server]: Failed to load the " << planner_type
                                                                    << " planner, are you sure it's properly registered"
                                                                    << " and that the containing library is built? "
                                                                    << ex.what());
  }
  return planner_ptr;
}

global_planner::AbstractPlannerExecution::Ptr PlannerActionServer::loadPlannerExecutionPlugin(const std::string &planner_execution_type) {
  global_planner::AbstractPlannerExecution::Ptr planner_execution_ptr;
  try {
    planner_execution_ptr = boost::static_pointer_cast<global_planner::AbstractPlannerExecution>(
        planner_execution_plugin_loader_.createInstance(planner_execution_type));
    std::string planner_execution_name = planner_execution_plugin_loader_.getName(planner_execution_type);
    ROS_DEBUG_STREAM(
        "[Planner Action Server]: Planner Execution plugin " << planner_execution_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("[Planner Action Server]: Failed to load the " << planner_execution_type
                                                                    << " planner execution, are you sure it's properly registered"
                                                                    << " and that the containing library is built? "
                                                                    << ex.what());
  }
  return planner_execution_ptr;
}

bool PlannerActionServer::initializePlannerPlugin(const std::string &name,
                                                  const global_planner::AbstractPlanner::Ptr &planner_ptr) {
  global_planner::AbstractPlanner::Ptr
      planner_pointer = boost::static_pointer_cast<global_planner::AbstractPlanner>(planner_ptr);
  ROS_DEBUG_STREAM("[Planner Action Server]: Initialize planner \"" << name << "\".");
  planner_pointer->initialize(name);
  ROS_DEBUG("[Planner Action Server]: Planner plugin initialized.");
  return true;
}

bool PlannerActionServer::initializePlannerExecutionPlugin(const std::string &name,
                                                           const global_planner::AbstractPlannerExecution::Ptr &planner_execution_ptr) {
  global_planner::AbstractPlannerExecution::Ptr
      planner_execution_pointer =
      boost::static_pointer_cast<global_planner::AbstractPlannerExecution>(planner_execution_ptr);
  ROS_DEBUG_STREAM("[Planner Action Server]: Initialize planner execution\"" << name << "\".");
  planner_execution_pointer->initialize(name, planner_plugin_);
  ROS_DEBUG("[Planner Action Server]: Planner Execution plugin initialized.");
  return true;
}

} // namespace global_planner
