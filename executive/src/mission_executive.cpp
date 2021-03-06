#include <executive/mission_executive.h>

namespace mission_executive {

MissionExecutive::MissionExecutive() : private_nh_("~"),
                                       controller_("/controller_executive/exe_path", true),
                                       planner_("/global_planner/get_path", true),
                                       recovery_("/recovery_executive/recovery", true),
                                       current_planner_goal_(nullptr),
                                       current_controller_goal_(nullptr),
                                       action_server_running_(false),
                                       is_controlling(false),
                                       recovery_name_("clear_costmap") {

  // creating call back queue for diagnostics pause and resume interrupt
  nh_.setCallbackQueue(&callback_queue_);
  private_nh_.setCallbackQueue(&callback_queue_);

  private_nh_.param("map_frame", map_frame_, std::string("map"));
  private_nh_.param("robot_frame", robot_frame_, std::string("base_link"));
  private_nh_.param("loop_frequency", loop_frequency_, 20.0);
  private_nh_.param("max_time_lag", max_time_lag_, 0.5);
  private_nh_.param("decay", decay_, 0.9);
  private_nh_.param("planner", planner_name_, std::string("SBPL"));
  private_nh_.param("controller", controller_name_, std::string("trajectory_planner"));
  private_nh_.param("plan_topic", plan_topic_, std::string("/plan"));
  private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  private_nh_.param("route_topic", route_topic_, std::string("/route"));
  private_nh_.param("goal_queue_topic", goal_queue_topic_, std::string("/goal_queue"));
  private_nh_.param("tf_timeout", tf_timeout_, 2.0);
  private_nh_.param("max_retries", max_retries_, 2);
  private_nh_.param("min_distance_tolerance", min_distance_tolerance_, 0.05);
  private_nh_.param("min_angular_tolerance", min_angular_tolerance_, 0.15);
  private_nh_.param("mission_status", mission_status_topic_, std::string("/mission_status"));

  double timeout;
  private_nh_.param("controller_reset_timeout", timeout, 10.0);
  controller_reset_timeout_ = ros::Duration(timeout);
  controller_reset_tries_ = 0;

  recovery_handler_ = std::make_shared<RecoveryHandler>(private_nh_);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  plan_pub_ = nh_.advertise<nav_msgs::Path>(plan_topic_, 1);
  route_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(route_topic_, 1);
  goal_queue_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(goal_queue_topic_, 1);
  mission_executive_status_pub_ = nh_.advertise<mw_msgs::mission_status>(mission_status_topic_, 1);
  
  goal_point_marker_.header.frame_id = map_frame_;
  goal_point_marker_.header.stamp = ros::Time();
  goal_point_marker_.ns = "goal_queue";
  goal_point_marker_.id = 0;
  goal_point_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  goal_point_marker_.action = visualization_msgs::Marker::ADD;
  goal_point_marker_.pose.orientation.w = 1.0;
  goal_point_marker_.scale.x = 0.15;
  goal_point_marker_.scale.y = 0.15;
  goal_point_marker_.color.r = 0.0f;
  goal_point_marker_.color.g = 0.0f;
  goal_point_marker_.color.b = 1.0f;
  goal_point_marker_.color.a = 1.0f;
  goal_point_marker_.frame_locked = (unsigned char) true;

  goal_text_marker_.header.frame_id = map_frame_;
  goal_text_marker_.header.stamp = ros::Time();
  goal_text_marker_.ns = "goal_queue";
  goal_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  goal_text_marker_.pose.orientation.w = 1.0;
  goal_text_marker_.scale.z = 0.15;
  goal_text_marker_.color.r = 1.0f;
  goal_text_marker_.color.g = 1.0f;
  goal_text_marker_.color.b = 1.0f;
  goal_text_marker_.color.a = 1.0f;
  goal_text_marker_.frame_locked = (unsigned char) true;

  cmd_vel_zero_.linear.x = 0;
  cmd_vel_zero_.linear.y = 0;
  cmd_vel_zero_.linear.z = 0;
  cmd_vel_zero_.angular.x = 0;
  cmd_vel_zero_.angular.y = 0;
  cmd_vel_zero_.angular.z = 0;

  ROS_WARN("[Mission Executive]: waiting for planning server");
  planner_.waitForServer();
  ROS_INFO("[Mission Executive]: Planning server is up and running");

  ROS_WARN("[Mission Executive]: waiting for controller server");
  controller_.waitForServer();
  ROS_INFO("[Mission Executive]: Controller server is up and running");

  ROS_WARN("[Mission Executive]: waiting for recovery server");
  recovery_.waitForServer();
  ROS_INFO("[Mission Executive]: Recovery server is up and running");

  state_ = WAITING;

  progress_checker_ = std::make_shared<mw_core::ProgressChecker>(min_distance_tolerance_, min_angular_tolerance_);

  set_plan_ = private_nh_.advertiseService("set_plan", &MissionExecutive::setCurrentPlan, this);
  set_route_ = private_nh_.advertiseService("set_route", &MissionExecutive::setCurrentRoute, this);
  route_status_ = private_nh_.advertiseService("get_route_status", &MissionExecutive::routeStatus, this);
  change_planner_ = private_nh_.advertiseService("change_planner", &MissionExecutive::changePlanner, this);
  change_controller_ =
      private_nh_.advertiseService("change_controller", &MissionExecutive::changeController, this);
  abort_planner_goals_ =
      private_nh_.advertiseService("abort_planner_goals", &MissionExecutive::abortPlannerGoals, this);
  abort_controller_goals_ =
      private_nh_.advertiseService("abort_controller_goals", &MissionExecutive::abortControllerGoals, this);
  abort_mission_ =
      private_nh_.advertiseService("abort_mission", &MissionExecutive::abortMission, this);
  trigger_recovery_ =
    private_nh_.advertiseService("trigger_recovery", &MissionExecutive::triggerRecovery, this);

  current_route_.status = mw_msgs::Route::NOT_STARTED;
  auto_pause_ = false;

#ifdef USE_OLD_TF
  ros::NodeHandle nh("mission_executive");
  tf_listener_ptr_.reset(new TF(nh, ros::Duration(10), true));
#else
  tf_listener_ptr_.reset(new TF(ros::Duration(10)));
  tf_listener_ = new tf2_ros::TransformListener(*tf_listener_ptr_);
#endif

  execute_timer_ =
      nh_.createTimer(ros::Duration(1 / loop_frequency_), boost::bind(&MissionExecutive::executeCB, this, _1));

  action_server_ = ActionServerWaypointNavigationPtr(
      new ActionServerWaypointNavigation(
          private_nh_,
          "waypoint_navigation",
          boost::bind(&MissionExecutive::callActionWaypointNavigation, this, _1),
          false));

  rviz_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &MissionExecutive::rvizGoalCB, this);

  action_server_->start();
}

MissionExecutive::~MissionExecutive() {
  execute_timer_.stop();
}

bool MissionExecutive::setCurrentRoute(mw_msgs::SetRoute::Request &req, mw_msgs::SetRoute::Response &resp) {
  ROS_DEBUG("[Mission Executive]: Set route called! Adding route to planner queue!");
  mw_msgs::GetPathGoal goal;
  goal.concurrency_slot = 0;
  goal.planner = planner_name_;
  goal.planner_execution = planner_name_ + "_execution";
  goal.use_current_pose = (unsigned char) false;
  goal.waypoints = req.route.waypoints;
  planner_goal_queue_.push_back(goal);
  resp.success = (unsigned char) true;
  publishGoalQueue();
  return true;
}

bool MissionExecutive::routeStatus(mw_msgs::GetRouteStatus::Request &req,
                                   mw_msgs::GetRouteStatus::Response &resp) {
  auto_pause_ = req.pause_state;
  resp.status = current_route_.status;
  return true;
}

bool MissionExecutive::setCurrentPlan(mw_msgs::SetPlan::Request &req, mw_msgs::SetPlan::Response &resp) {
  ROS_INFO("[Mission Executive]: Set plan called! Adding plan to controller queue!");
  mw_msgs::ExePathGoal goal;
  goal.concurrency_slot = 0;
  goal.controller = controller_name_;
  goal.controller_execution = controller_name_ + "_execution";
  goal.path.header.stamp = ros::Time::now();
  goal.path.header.frame_id = map_frame_;
  goal.path.poses.clear();
  for (auto &pose : req.plan.poses) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose = pose;
    poseStamped.header = goal.path.header;
    goal.path.poses.push_back(poseStamped);
  }
  controller_goal_queue_.push_back(goal);
  resp.success = (unsigned char) true;
  return true;
}

void MissionExecutive::activePlannerCB() {
  ROS_INFO("[Mission Executive]: Planning using %s", planner_name_.c_str());
  state_ = PLANNING;
  current_route_.status = mw_msgs::Route::WIP;
}

void MissionExecutive::donePlannerCB(const actionlib::SimpleClientGoalState &state,
                                     const mw_msgs::GetPathResultConstPtr &result) {
  if (state.state_ == state.SUCCEEDED) {
    ROS_INFO("[Mission Executive]: The planning action has succeeded");

    if (result->outcome < 10) {
      ROS_INFO("[Mission Executive]: The Planning was successful");
      ROS_DEBUG("[Mission Executive]: The plan received is of size %ld and cost %lf",
                result->path.poses.size(),
                result->cost);
      ROS_DEBUG("[Mission Executive]: Adding plan to controller queue!");

      global_plan_.header.stamp = ros::Time::now();
      global_plan_.header.frame_id = map_frame_;
      global_plan_ = result->path;
      plan_pub_.publish(global_plan_);

      mw_msgs::ExePathGoal goal;
      goal.concurrency_slot = 0;
      goal.controller = controller_name_;
      goal.controller_execution = controller_name_ + "_execution";
      goal.path = result->path;
      controller_goal_queue_.push_back(goal);
      return;
    } else {
      ROS_WARN("[Mission Executive]: Planning was not successful! The status of the plan is: %s",
               result->message.c_str());
      global_plan_.poses.clear();
    }
  } else {
    ROS_WARN("[Mission Executive]: The planning action didn't succeed! The state of the planner was %s ",
             state.toString().c_str());
    global_plan_.poses.clear();
  }

  if (action_server_running_) {
    action_server_->setAborted(mw_msgs::WaypointNavigationResult(), "Global planner failed to plan a path!");
    action_server_running_ = false;
  }

  state_ = WAITING;
}

void MissionExecutive::activeControllerCB() {
  ROS_INFO("[Mission Executive]: Controlling using %s", controller_name_.c_str());
  state_ = CONTROLLING;
  current_route_.status = mw_msgs::Route::WIP;
  last_feedback_time_ = ros::Time::now();
  progress_checker_->reset();
}

void MissionExecutive::doneControllerCB(const actionlib::SimpleClientGoalState &state,
                                        const mw_msgs::ExePathResultConstPtr &result) {
  cmd_vel_ = cmd_vel_zero_;
  state_ = WAITING;
  error_code_ = result->outcome;
  message_ = result->message;
  if (state.state_ == state.SUCCEEDED) {
    ROS_INFO("[Mission Executive]: The controller finished in state [%s] ", state.toString().c_str());
    current_route_.status = mw_msgs::Route::COMPLETED;
    distance_to_goal_ = result->dist_to_goal;
    current_planner_goal_ = nullptr;
    current_controller_goal_ = nullptr;
    if (action_server_running_) {
      action_server_->setSucceeded(mw_msgs::WaypointNavigationResult(), result->message);
      action_server_running_ = false;
    }
  } else {
    ROS_WARN("[Mission Executive]: The controller finished in state [%s] ", state.toString().c_str());
    if (action_server_running_) {
      action_server_->setAborted(mw_msgs::WaypointNavigationResult(),
                                 "Controller failed to reach goal (error: " + result->message + ")");
      action_server_running_ = false;
    }
  }
}

void MissionExecutive::feedbackControllerCB(const mw_msgs::ExePathFeedbackConstPtr &feedback) {
  ROS_DEBUG("[Mission Executive]: Received feedback from controller with message: %s", feedback->message.c_str());
  // ROS_INFO("[Mission Executive]: Received feedback from controller with message: %s", feedback->message.c_str());
  cmd_vel_ = feedback->last_cmd_vel.twist;
  distance_to_goal_ = feedback->dist_to_goal;
  error_code_ = feedback->outcome;
  message_ = feedback->message;
  last_feedback_time_ = ros::Time::now();
  current_route_.status = mw_msgs::Route::WIP;
}

void MissionExecutive::activeRecoveryCB() {
  ROS_INFO("[Mission Executive]: Recovering using %s", recovery_name_.c_str());
  state_ = RECOVERING;
}

void MissionExecutive::doneRecoveryCB(const actionlib::SimpleClientGoalState &state,
                                      const mw_msgs::RecoveryResultConstPtr &result) {
  state_ = is_controlling ? CONTROLLING : WAITING;
  error_code_ = result->outcome;
  message_ = result->message;
  if (state.state_ == state.SUCCEEDED) {
    ROS_INFO("[Mission Executive]: The recovery finished in state [%s] ", state.toString().c_str());
  } else {
    ROS_WARN("[Mission Executive]: The controller finished in state [%s] ", state.toString().c_str());
  }
}

void MissionExecutive::publishRouteMarker(const std::vector<geometry_msgs::PoseStamped> &waypoints) {
  visualization_msgs::Marker route_marker;
  if (!waypoints.empty())
    route_marker.header = waypoints[0].header;
  route_marker.action = visualization_msgs::Marker::ADD;
  route_marker.id = 0;
  route_marker.type = visualization_msgs::Marker::POINTS;
  route_marker.scale.x = 0.1;
  route_marker.scale.y = 0.1;
  route_marker.color.g = 1.0;
  route_marker.color.a = 1.0;
  for (auto &wp : waypoints) {
    geometry_msgs::Point p;
    p.x = wp.pose.position.x;
    p.y = wp.pose.position.y;
    p.z = wp.pose.position.z;
    route_marker.points.push_back(p);
  }
  route_marker_pub_.publish(route_marker);
}

void MissionExecutive::publishGoalQueue() {
  if (planner_goal_queue_.empty())
    return;

  goal_point_marker_.points.clear();
  goal_queue_marker_msg_.markers.clear();

  for (int i = 0; i < (int) planner_goal_queue_.size(); ++i) {
    goal_text_marker_.id = i + 1;
    goal_text_marker_.text = std::to_string(i + 1);
    goal_text_marker_.pose.position = planner_goal_queue_[i].waypoints[0].pose.position;
    goal_queue_marker_msg_.markers.push_back(goal_text_marker_);
    goal_point_marker_.points.push_back(planner_goal_queue_[i].waypoints[0].pose.position);
  }

  goal_text_marker_.id = (int) planner_goal_queue_.size() + 1;
  goal_text_marker_.text = "";
  goal_queue_marker_msg_.markers.push_back(goal_text_marker_);

  goal_queue_marker_msg_.markers.push_back(goal_point_marker_);
  goal_queue_pub_.publish(goal_queue_marker_msg_);
}

void MissionExecutive::executeCB(const ros::TimerEvent &event) {
  switch (state_) {
    case WAITING: {
      ROS_INFO_THROTTLE(5, "[Mission Executive]: waiting for route");
      is_controlling = false;
      cmd_vel_pub_.publish(cmd_vel_zero_);

      if (!planner_goal_queue_.empty()) {
        recovery_handler_->resetCount();
        controller_reset_tries_ = 0;
        current_route_.status = mw_msgs::Route::NOT_STARTED;
        current_planner_goal_.reset(new mw_msgs::GetPathGoal(planner_goal_queue_.front()));
        current_planner_goal_->planner = planner_name_;
        current_planner_goal_->planner_execution = planner_name_ + "_execution";
        planner_goal_queue_.pop_front();
        planner_.sendGoal(*current_planner_goal_,
                          boost::bind(&MissionExecutive::donePlannerCB, this, _1, _2),
                          boost::bind(&MissionExecutive::activePlannerCB, this));
        publishRouteMarker(current_planner_goal_->waypoints);
        publishGoalQueue();
      } else if (!controller_goal_queue_.empty()) {
        ROS_WARN("[Mission Executive]: Planner queue is empty but controller queue contains a path!");
        recovery_handler_->resetCount();
        controller_reset_tries_ = 0;
        current_controller_goal_.reset(new mw_msgs::ExePathGoal(controller_goal_queue_.front()));
        current_controller_goal_->controller = controller_name_;
        current_controller_goal_->controller_execution = controller_name_ + "_execution";
        controller_goal_queue_.pop_front();
        controller_.sendGoal(*current_controller_goal_,
                             boost::bind(&MissionExecutive::doneControllerCB, this, _1, _2),
                             boost::bind(&MissionExecutive::activeControllerCB, this),
                             boost::bind(&MissionExecutive::feedbackControllerCB, this, _1));

        global_plan_.header.stamp = ros::Time::now();
        global_plan_.header.frame_id = map_frame_;
        global_plan_ = current_controller_goal_->path;
        plan_pub_.publish(global_plan_);
      }

      break;
    }

    case PLANNING: {
      ROS_INFO_THROTTLE(5, "[Mission Executive]: planning");
      is_controlling = false;
      cmd_vel_pub_.publish(cmd_vel_zero_);
      if (!controller_goal_queue_.empty()) {
        current_controller_goal_.reset(new mw_msgs::ExePathGoal(controller_goal_queue_.front()));
        current_controller_goal_->controller = controller_name_;
        current_controller_goal_->controller_execution = controller_name_ + "_execution";
        controller_goal_queue_.pop_front();
        controller_.sendGoal(*current_controller_goal_,
                             boost::bind(&MissionExecutive::doneControllerCB, this, _1, _2),
                             boost::bind(&MissionExecutive::activeControllerCB, this),
                             boost::bind(&MissionExecutive::feedbackControllerCB, this, _1));
      }
      break;
    }

    case CONTROLLING: {

      is_controlling = true;

      if (auto_pause_) {
        ROS_INFO_THROTTLE(5, "[Mission Executive]: mission is paused");
        cmd_vel_pub_.publish(cmd_vel_zero_);
        break;
      }

      ROS_INFO_THROTTLE(5, "[Mission Executive]: controlling");

      if (ros::Time::now().toSec() - last_feedback_time_.toSec() < max_time_lag_) {
        if (error_code_ == 0) {
          ROS_DEBUG("[Mission Executive]: Velocities linear: %f  angular: %f",
                    cmd_vel_.linear.x,
                    cmd_vel_.angular.z);
        } else {
          ROS_ERROR_THROTTLE(2.0,
                             "[Mission Executive]: %s (error code: %d)",
                             message_.c_str(),
                             error_code_);
          cmd_vel_ = cmd_vel_zero_;
        }
      } else {
        ROS_INFO_THROTTLE(5,
                          "[Mission Executive]: not receiving feedback from controller, decaying last command!");
        cmd_vel_.linear.x *= decay_;
        cmd_vel_.linear.y *= decay_;
        cmd_vel_.linear.z *= decay_;
        cmd_vel_.angular.x *= decay_;
        cmd_vel_.angular.y *= decay_;
        cmd_vel_.angular.z *= decay_;
      }

      // cmd_vel_pub_.publish(cmd_vel_);

      // Get the current pose
      if (!mw_core::getRobotPose(*tf_listener_ptr_, robot_frame_, map_frame_,
                                 ros::Duration(tf_timeout_), current_pose_)) {
        ROS_WARN_STREAM(
          "[Mission Executive]: Could not get the robot pose in the global frame. - robot frame: \""
            << robot_frame_ << "\"   global frame: \"" << map_frame_ << std::endl);

        if (ros::Time::now() - current_pose_.header.stamp > controller_reset_timeout_) {
          ROS_ERROR(
            "[Mission Executive]: Resetting mission executive since transforms were not available for %lf secs",
            controller_reset_timeout_.toSec());
          reset();
        }
        break;
      }

      current_pose_.header.stamp = ros::Time::now();

      double progress_timeout = progress_checker_->check(current_pose_);

      if (recovery_handler_->triggerRecovery(progress_timeout, recovery_name_)) {
        ROS_WARN("[Mission Executive]: Robot has not moved for %lf seconds", progress_timeout);
        sendRecoveryGoal();
      }

      if (progress_timeout > controller_reset_timeout_.toSec() * (controller_reset_tries_ + 1)) {
        ROS_WARN("[Mission Executive]: Robot has not moved for %lf seconds", progress_timeout);
        controller_reset_tries_++;
        planner_.cancelAllGoals();
        controller_.cancelAllGoals();
        planner_.waitForResult();
        controller_.waitForResult();
        current_controller_goal_ = nullptr;
        if (current_planner_goal_ != nullptr) {
          ROS_WARN("[Mission Executive]: Replanning last route!");
          state_ = PLANNING;
          current_route_.status = mw_msgs::Route::NOT_STARTED;
          current_planner_goal_->planner = planner_name_;
          current_planner_goal_->planner_execution = planner_name_ + "_execution";
          current_planner_goal_->use_current_pose = (unsigned char) true;
          planner_.sendGoal(*current_planner_goal_,
                            boost::bind(&MissionExecutive::donePlannerCB, this, _1, _2),
                            boost::bind(&MissionExecutive::activePlannerCB, this));
          publishRouteMarker(current_planner_goal_->waypoints);
        }
      }

      if (controller_reset_tries_ > max_retries_) {
        ROS_WARN("[Mission Executive]: Max retries (%d) reached, aborting mission!", max_retries_);
        current_planner_goal_ = nullptr;
        state_ = WAITING;
        controller_reset_tries_ = 0;
        recovery_handler_->resetCount();
        return;
      }

      break;
    }

    case RECOVERING: {
      ROS_INFO_THROTTLE(5, "[Mission Executive]: recovering");
      break;
    }

    default:
      ROS_ERROR("[Mission Executive]: should not be here!");
  }
}

void MissionExecutive::sendRecoveryGoal() {
  cmd_vel_pub_.publish(cmd_vel_zero_);
  mw_msgs::RecoveryGoal recovery_goal;
  recovery_goal.behavior = recovery_name_;
  recovery_goal.behavior_execution = recovery_name_ + "_execution";
  recovery_goal.concurrency_slot = 0;
  recovery_.sendGoal(recovery_goal,
                     boost::bind(&MissionExecutive::doneRecoveryCB, this, _1, _2),
                     boost::bind(&MissionExecutive::activeRecoveryCB, this));
}

void MissionExecutive::spinClass() {
  callback_queue_.callAvailable(ros::WallDuration());
}

void MissionExecutive::rvizGoalCB(const geometry_msgs::PoseStamped::Ptr &msg) {
  mw_msgs::GetPathGoal goal;
  goal.use_current_pose = (unsigned char) true;
  goal.tolerance = 0.5;
  goal.planner = planner_name_;
  goal.planner_execution = planner_name_ + "_execution";
  goal.concurrency_slot = 0;
  goal.waypoints.clear();
  goal.waypoints.push_back(*msg);
  planner_goal_queue_.push_back(goal);
  publishGoalQueue();
  ROS_INFO("[Mission Executive]: Goal pushed to the planner queue!");
}

bool MissionExecutive::changePlanner(mw_msgs::SetString::Request &req, mw_msgs::SetString::Response &resp) {
  ROS_INFO("[Mission Executive]: Changed planner to %s", req.value.c_str());
  planner_name_ = req.value;
  resp.status = (unsigned char) true;
  return true;
}

bool MissionExecutive::changeController(mw_msgs::SetString::Request &req, mw_msgs::SetString::Response &resp) {
  ROS_INFO("[Mission Executive]: Changed controller to %s", req.value.c_str());
  controller_name_ = req.value;
  resp.status = (unsigned char) true;
  return true;
}

bool MissionExecutive::abortPlannerGoals(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ROS_INFO("[Mission Executive]: Cancelled all planner goals!");
  planner_.cancelAllGoals();
  current_planner_goal_ = nullptr;
  state_ = WAITING;
  return true;
}

bool MissionExecutive::abortControllerGoals(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ROS_INFO("[Mission Executive]: Cancelled all controller goals!");
  controller_.cancelAllGoals();
  current_controller_goal_ = nullptr;
  state_ = WAITING;
  return true;
}

bool MissionExecutive::abortMission(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ROS_INFO("[Mission Executive]: Mission was aborted!");
  reset();
  return true;
}

bool MissionExecutive::triggerRecovery(mw_msgs::SetString::Request &req, mw_msgs::SetString::Response &resp) {
  ROS_INFO("[Mission Executive]: Triggering recovery %s", req.value.c_str());
  recovery_name_ = req.value;
  sendRecoveryGoal();
  resp.status = (unsigned char) true;
  return true;
}

void MissionExecutive::clearPlannerQueue() {
  planner_goal_queue_.clear();
}

void MissionExecutive::clearControllerQueue() {
  controller_goal_queue_.clear();
}

void MissionExecutive::reset() {
  planner_.cancelAllGoals();
  controller_.cancelAllGoals();
  recovery_.cancelAllGoals();
  current_planner_goal_ = nullptr;
  current_controller_goal_ = nullptr;
  clearPlannerQueue();
  clearControllerQueue();
  state_ = WAITING;
}

void MissionExecutive::callActionWaypointNavigation(const ActionServerWaypointNavigation::GoalConstPtr &new_goal) {
  ROS_INFO("[Mission Executive]: Called action \"waypoint_navigation\"");

  if (action_server_running_) {
    ROS_ERROR(
        "[Mission Executive]: Action server is already running a goal, there is some problem! Resetting for safety.");
    //if the node is killed then we'll abort and return
    action_server_->setAborted(mw_msgs::WaypointNavigationResult(), "A goal was already being handled");
    action_server_running_ = false;
    reset();
    return;
  }

  geometry_msgs::PoseStamped goal(new_goal->target_pose);
  ros::Rate rate(loop_frequency_);

  // loop till state is waiting and planner and controller queues are empty
  while (ros::ok()) {
    if (action_server_->isPreemptRequested()) {
      if (action_server_->isNewGoalAvailable()) {
        // if we're active and a new goal is available, we'll accept it, but we won't shut anything down
        mw_msgs::WaypointNavigationGoal _new_goal = *action_server_->acceptNewGoal();
        goal = _new_goal.target_pose;
        // publish the goal point to the visualizer
        ROS_DEBUG("[Mission Executive]: received a goal of x: %.2f, y: %.2f",
                  goal.pose.position.x,
                  goal.pose.position.y);
      } else {
        //notify the ActionServer that we've successfully preempted
        ROS_DEBUG("[Mission Executive]: preempting the current goal");
        action_server_->setPreempted();
        //we'll actually return from execute after preempting
        return;
      }
    }

    if (state_ == WAITING && planner_goal_queue_.empty() && controller_goal_queue_.empty()) {
      mw_msgs::GetPathGoal planner_goal;
      planner_goal.concurrency_slot = 0;
      planner_goal.planner = planner_name_;
      planner_goal.planner_execution = planner_name_ + "_execution";
      planner_goal.tolerance = 0.5;
      planner_goal.use_current_pose = (unsigned char) true;
      planner_goal.waypoints.push_back(goal);
      planner_goal_queue_.push_back(planner_goal);
      action_server_running_ = true;
      publishGoalQueue();
      ROS_INFO("[Mission Executive]: Goal pushed to planner queue! (x: %lf, y: %lf)",
               goal.pose.position.x,
               goal.pose.position.y);
      break;
    }

    ROS_WARN_THROTTLE(5,
                      "[Mission Executive]: \"waypoint_navigation\" action server is waiting for mission executive to get free...");
    rate.sleep();
  }

  while (ros::ok()) {
    if (action_server_->isPreemptRequested()) {
      if (action_server_running_) {
        ROS_WARN("[Mission Executive]: preempt requested, resetting the mission executive!");
        reset();
      }
      if (action_server_->isNewGoalAvailable()) {
        // if we're active and a new goal is available, we'll accept it, but we won't shut anything down
        mw_msgs::WaypointNavigationGoal _new_goal = *action_server_->acceptNewGoal();
        goal = _new_goal.target_pose;

        // publish the goal point to the visualizer
        ROS_DEBUG("[Mission Executive]: received a goal of x: %.2f, y: %.2f",
                  goal.pose.position.x,
                  goal.pose.position.y);

        mw_msgs::GetPathGoal planner_goal;
        planner_goal.concurrency_slot = 0;
        planner_goal.planner = planner_name_;
        planner_goal.planner_execution = planner_name_ + "_execution";
        planner_goal.tolerance = 0.5;
        planner_goal.use_current_pose = (unsigned char) true;
        planner_goal.waypoints.push_back(goal);
        planner_goal_queue_.push_back(planner_goal);
        publishGoalQueue();
        ROS_INFO("[Mission Executive]: Goal pushed to planner queue! (x: %lf, y: %lf)",
                 goal.pose.position.x,
                 goal.pose.position.y);
        action_server_running_ = true;
      } else {
        //notify the ActionServer that we've successfully preempted
        ROS_DEBUG("[Mission Executive]: preempting the current goal");
        action_server_->setPreempted();
        //we'll actually return from execute after preempting
        return;
      }
    }

    // if we're done, then we'll return from execute
    if (!action_server_running_)
      return;

    ROS_WARN_THROTTLE(5, "[Mission Executive]: \"waypoint_navigation\" action server is running...");
    rate.sleep();
  }

  //if the node is killed then we'll abort and return
  action_server_->setAborted(mw_msgs::WaypointNavigationResult(),
                             "Aborting on the goal because the node has been killed");
  action_server_running_ = false;
}

// get enum values representing state of the mission
MissionExecutive::MissionExecutiveState MissionExecutive::getMissionExecState(){
  return state_;
}

// get enum values to string representation for the state of the mission
std::string MissionExecutive::getMissionExecStateName(){
  switch(state_){
    case MissionExecutive::MissionExecutiveState::WAITING:
      return "WAITING";

    case MissionExecutive::MissionExecutiveState::PLANNING:
      return "PLANNING";

    case MissionExecutive::MissionExecutiveState::CONTROLLING:
      return "CONTROLLING";

    case MissionExecutive::MissionExecutiveState::RECOVERING:
      return "RECOVERING";

    default:
      return "UNKNOWN";
  }
}

// get mission_status message (state, controller type/name, global type/name)
mw_msgs::mission_status MissionExecutive::getMissionStatus(){
  mw_msgs::mission_status msg;
  msg.machine_state = MissionExecutive::getMissionExecStateName();
  msg.controller_type = controller_name_;
  msg.global_planner_type = planner_name_;
  return msg;
}

// publisher for message mission_status
void MissionExecutive::startMissionExecutiveStatusPublisher(){
  mw_msgs::mission_status msg = getMissionStatus();
  mission_executive_status_pub_.publish(msg);
}

} // namespace executive
