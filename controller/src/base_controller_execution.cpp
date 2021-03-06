/**
 *
 * @file base_controller_execution.cpp
 * @brief The base controller execution class. Every controller execution must inherit this class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "controller_executive/base_controller_execution.h"

namespace controller {

const double AbstractControllerExecution::DEFAULT_CONTROLLER_FREQUENCY = 10.0; // 100 Hz

AbstractControllerExecution::AbstractControllerExecution() : AbstractExecution(),
                                                             state_(INITIALIZED),
                                                             moving_(false),
                                                             max_retries_(0),
                                                             patience_(0),
                                                             calling_duration_(boost::chrono::microseconds(static_cast<int>(
                                                                                                               1e6
                                                                                                                   / DEFAULT_CONTROLLER_FREQUENCY))) {

  ros::NodeHandle nh;
  nh.setCallbackQueue(&callback_queue_);

  ros::NodeHandle private_nh("~");

  // non-dynamically reconfigurable parameters
  private_nh.param("odom_topic", odom_topic_, std::string("/odom"));
  private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
  private_nh.param("map_frame", global_frame_, std::string("map"));
  private_nh.param("tolerance_check", tolerance_check_, false);
  private_nh.param("force_stop_at_goal", force_stop_at_goal_, true);
  private_nh.param("dist_tolerance", dist_tolerance_, 0.1);
  private_nh.param("angle_tolerance", angle_tolerance_, M_PI / 18.0);
  private_nh.param("tf_timeout", tf_timeout_, 1.0);

  odom_sub_ = nh.subscribe(odom_topic_, 1, &AbstractControllerExecution::odometryCallback, this);
}

AbstractControllerExecution::~AbstractControllerExecution() {
}

void AbstractControllerExecution::initialize(const std::string &name,
                                             const controller::AbstractController::Ptr &controller_ptr,
                                             const TFPtr &tf_listener_ptr) {
  AbstractExecution::initialize(name);
  ros::NodeHandle private_nh("~/" + name);
  tf_listener_ptr_ = tf_listener_ptr;
  vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
  controller_ = controller_ptr;
  ROS_DEBUG("[Controller Execution]: Abstract Controller Execution initialized!");
}

bool AbstractControllerExecution::setControllerFrequency(double frequency) {
  // set the calling duration by the moving frequency
  if (frequency <= 0.0) {
    ROS_ERROR("[Controller Execution]: Controller frequency must be greater than 0.0! No change of the frequency!");
    return false;
  }
  calling_duration_ = boost::chrono::microseconds(static_cast<int>(1e6 / frequency));
  return true;
}

bool AbstractControllerExecution::start() {
  setState(STARTED);
  if (moving_) {
    ROS_DEBUG("[Controller Execution]: start called but thread is already running");
    return false; // thread is already running.
  }
  moving_ = true;
  ROS_DEBUG("[Controller Execution]: controller execution started");
  return AbstractExecution::start();
}

void AbstractControllerExecution::postRun() {
  reset();
}

bool AbstractControllerExecution::reset() {
  setState(INITIALIZED);
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  plan_.clear();
  cancel_ = false;
  moving_ = false;
  controller_->reset();
  ROS_DEBUG("[Controller Execution]: controller execution was reset");
  return true;
}

void AbstractControllerExecution::setState(ControllerState state) {
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  state_ = state;
  ROS_DEBUG_THROTTLE(1, "[Controller Execution]: changed state to %d", state);
}

typename AbstractControllerExecution::ControllerState
AbstractControllerExecution::getState() {
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  return state_;
}

void AbstractControllerExecution::setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  if (moving_) {
    // This is fine on continuous replanning
    ROS_DEBUG("[Controller Execution]: Setting new plan while moving");
    reset();
  }
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  new_plan_ = true;
  ROS_DEBUG("[Controller Execution]: New plan was set!");
  plan_ = plan;
}

bool AbstractControllerExecution::hasNewPlan() {
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  return new_plan_;
}

std::vector<geometry_msgs::PoseStamped> AbstractControllerExecution::getNewPlan() {
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  new_plan_ = false;
  return plan_;
}

void AbstractControllerExecution::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  robot_velocity_.header = msg->header;
  robot_velocity_.twist = msg->twist.twist;
}

bool AbstractControllerExecution::computeRobotPose() {
  bool tf_success = mw_core::getRobotPose(*tf_listener_ptr_, robot_frame_, global_frame_,
                                          ros::Duration(tf_timeout_), robot_pose_);
  // would be 0 if not, as we ask tf listener for the last pose available
  robot_pose_.header.stamp = ros::Time::now();
  if (!tf_success) {
    ROS_ERROR_STREAM("[Controller Execution]: Could not get the robot pose in the global frame. - robot frame: \""
                         << robot_frame_ << "\"   global frame: \"" << global_frame_ << std::endl);
    message_ = "[Controller Execution]: Could not get the robot pose";
    outcome_ = 255;
    return false;
  }
  return true;
}

uint32_t AbstractControllerExecution::computeVelocityCmd(const geometry_msgs::PoseStamped &robot_pose,
                                                         const geometry_msgs::TwistStamped &robot_velocity,
                                                         geometry_msgs::TwistStamped &vel_cmd,
                                                         std::string &message) {
  uint32_t outcome = controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
  if (outcome != 0 && message.empty())
    message = "Controller plugin was not able to calculate a velocity command!";
  return outcome;
}

void AbstractControllerExecution::setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd) {
  boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
  vel_cmd_stamped_ = vel_cmd;
  if (vel_cmd_stamped_.header.stamp.isZero())
    vel_cmd_stamped_.header.stamp = ros::Time::now();
  // TODO what happen with frame id?
  // TODO Add a queue here for handling the outcome, message and cmd_vel values bundled,
  // TODO so there should be no loss of information in the feedback stream
}

geometry_msgs::TwistStamped AbstractControllerExecution::getVelocityCmd() {
  boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
  return vel_cmd_stamped_;
}

ros::Time AbstractControllerExecution::getLastPluginCallTime() {
  boost::lock_guard<boost::mutex> guard(lct_mtx_);
  return last_call_time_;
}

bool AbstractControllerExecution::isPatienceExceeded() {
  boost::lock_guard<boost::mutex> guard(lct_mtx_);
  return !patience_.isZero() && (ros::Time::now() - last_call_time_ > patience_);
}

bool AbstractControllerExecution::isMoving() {
  return moving_;
}

bool AbstractControllerExecution::reachedGoalCheck() {
  // check whether the controller plugin returns goal reached or if mbf should check for goal reached.
  return controller_->isGoalReached(dist_tolerance_, angle_tolerance_) || (tolerance_check_
      && mw_core::distance(robot_pose_, plan_.back()) < dist_tolerance_
      && mw_core::angle(robot_pose_, plan_.back()) < angle_tolerance_);
}

bool AbstractControllerExecution::cancel() {
  // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
  bool ctrl_cancelled = controller_->cancel();
  if (!ctrl_cancelled) {
    ROS_WARN_STREAM("[Controller Execution]: Cancel controlling failed or is not supported by the plugin. "
                        << "Wait until the current control cycle finished!");
  }
  cancel_ = true;
  return ctrl_cancelled;
}

void AbstractControllerExecution::run() {
  start_time_ = ros::Time::now();

  // init plan
  std::vector<geometry_msgs::PoseStamped> plan;
  if (!hasNewPlan()) {
    setState(NO_PLAN);
    moving_ = false;
    ROS_ERROR("[Controller Execution]: robot navigation moving has no plan!");
  }

  ros::Time last_valid_cmd_time = ros::Time();
  int retries = 0;
  unsigned int seq = 0;

  try {
    while (moving_ && ros::ok()) {
      boost::chrono::thread_clock::time_point loop_start_time = boost::chrono::thread_clock::now();

      callback_queue_.callAvailable();
      geometry_msgs::TwistStamped cmd_vel_stamped;

      if (cancel_) {
        setZeroVelocity(cmd_vel_stamped); // command the robot to stop on canceling navigation
        vel_pub_.publish(cmd_vel_stamped.twist);
        setVelocityCmd(cmd_vel_stamped);
        setState(CANCELED);
        condition_.notify_all();
        moving_ = false;
        return;
      }

      if (!safetyCheck()) {
        // the specific implementation must have detected a risk situation; at this abstract level, we
        // cannot tell what the problem is, but anyway we command the robot to stop to avoid crashes
        setZeroVelocity(cmd_vel_stamped);   // note that we still feedback command calculated by the plugin
        boost::this_thread::sleep_for(calling_duration_);
      }

      // update plan dynamically
      if (hasNewPlan()) {
        plan = getNewPlan();

        // check if plan is empty
        if (plan.empty()) {
          setState(EMPTY_PLAN);
          condition_.notify_all();
          moving_ = false;
          return;
        }

        // check if plan could be set
        if (!controller_->setPlan(plan)) {
          setState(INVALID_PLAN);
          condition_.notify_all();
          moving_ = false;
          return;
        }
        current_goal_pub_.publish(plan.back());
      }

      // compute robot pose and store it in robot_pose_
      computeRobotPose();

      // ask planner if the goal is reached
      if (reachedGoalCheck()) {
        ROS_DEBUG("[Controller Execution]: Reached the goal!");
        if (force_stop_at_goal_) {
          setZeroVelocity(cmd_vel_stamped);
        }
        setState(ARRIVED_GOAL);
        // goal reached, tell it the controller
        condition_.notify_all();
        moving_ = false;
        // if not, keep moving
      } else {
        setState(PLANNING);

        // save time and call the plugin
        lct_mtx_.lock();
        last_call_time_ = ros::Time::now();
        lct_mtx_.unlock();

        // call plugin to compute the next velocity command
        outcome_ = computeVelocityCmd(robot_pose_, robot_velocity_, cmd_vel_stamped, message_ = "");

        if (outcome_ < 10) {
          setState(GOT_LOCAL_CMD);
          // ROS_INFO("[Controller executive base_controller_execution]:state set to GOT_LOCAL_CMD");
          vel_pub_.publish(cmd_vel_stamped.twist);
          last_valid_cmd_time = ros::Time::now();
          retries = 0;
          ROS_DEBUG_THROTTLE(1, "[Controller Execution]: Published cmd_vel: X: %lf, Y: %lf, Theta: %lf",
                             cmd_vel_stamped.twist.linear.x,
                             cmd_vel_stamped.twist.linear.y,
                             cmd_vel_stamped.twist.angular.z);
        } else {
          boost::lock_guard<boost::mutex> guard(configuration_mutex_);
          if (max_retries_ >= 0 && ++retries > max_retries_) {
            setState(MAX_RETRIES);
            moving_ = false;
          } else if (!patience_.isZero() && ros::Time::now() - last_valid_cmd_time > patience_
              && ros::Time::now() - start_time_ > patience_) {
            // patience limit enabled and running controller for more than patience without valid commands
            setState(PAT_EXCEEDED);
            moving_ = false;
          } else {
            setState(NO_LOCAL_CMD); // useful for server feedback
          }
          // could not compute a valid velocity command -> stop moving the robot
          setZeroVelocity(cmd_vel_stamped); // command the robot to stop; we still feedback command calculated by the plugin
        }

        // set stamped values; timestamp and frame_id should be set by the plugin; otherwise setVelocityCmd will do
        cmd_vel_stamped.header.seq = seq++; // sequence number
        setVelocityCmd(cmd_vel_stamped);
        condition_.notify_all();
      }

      boost::chrono::thread_clock::time_point end_time = boost::chrono::thread_clock::now();
      boost::chrono::microseconds execution_duration =
          boost::chrono::duration_cast<boost::chrono::microseconds>(end_time - loop_start_time);
      configuration_mutex_.lock();
      boost::chrono::microseconds sleep_time = calling_duration_ - execution_duration;
      configuration_mutex_.unlock();
      if (moving_ && ros::ok()) {
        if (sleep_time > boost::chrono::microseconds(0)) {
          // interruption point
          boost::this_thread::sleep_for(sleep_time);
        } else {
          // provide an interruption point also with 0 or negative sleep_time
          boost::this_thread::interruption_point();
          ROS_WARN_THROTTLE(1.0,
                            "[Controller Execution]: Calculation needs too much time to stay in the moving frequency! (%f > %f)",
                            execution_duration.count() / 1000000.0,
                            calling_duration_.count() / 1000000.0);
        }
      }
    }
  }
  catch (const boost::thread_interrupted &ex) {
    // Controller thread interrupted; in most cases we have started a new plan
    // Can also be that robot is oscillating or we have exceeded planner patience
    ROS_DEBUG("[Controller Execution]: Controller thread interrupted!");
    geometry_msgs::TwistStamped cmd_vel_stamped;
    setZeroVelocity(cmd_vel_stamped); // command the robot to stop on canceling navigation
    vel_pub_.publish(cmd_vel_stamped.twist);
    setVelocityCmd(cmd_vel_stamped);
    setState(STOPPED);
    condition_.notify_all();
    moving_ = false;
  }
  catch (...) {
    message_ = "[Controller Execution]: Unknown error occurred: " + boost::current_exception_diagnostic_information();
    ROS_FATAL_STREAM(message_);
    setState(INTERNAL_ERROR);
  }
}

void AbstractControllerExecution::setZeroVelocity(geometry_msgs::TwistStamped &cmd_vel_stamped) {
  cmd_vel_stamped.header.frame_id = "map";
  cmd_vel_stamped.header.stamp = ros::Time::now();
  cmd_vel_stamped.twist.linear.x = 0;
  cmd_vel_stamped.twist.linear.y = 0;
  cmd_vel_stamped.twist.linear.z = 0;
  cmd_vel_stamped.twist.angular.x = 0;
  cmd_vel_stamped.twist.angular.y = 0;
  cmd_vel_stamped.twist.angular.z = 0;
}

}